"""
surrogate.py — PyTorch MLP aerodynamic surrogate for the canard tailsitter VTOL.

Architecture (340k parameters, fully differentiable):
    Input(12) → Linear+BN+SiLU(128) → Linear+BN+SiLU(256)
              → ResBlock(256) → ResBlock(256)
              → Linear+BN+SiLU(128) → Linear+BN+SiLU(64)
              → Linear(3)   [CL, CD, Cm]

SiLU activations give smooth, non-zero gradients everywhere — critical for
gradient-based design optimization through the network.
"""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader, Subset

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Network definition
# ---------------------------------------------------------------------------

class _Block(nn.Sequential):
    def __init__(self, in_features: int, out_features: int) -> None:
        super().__init__(
            nn.Linear(in_features, out_features),
            nn.BatchNorm1d(out_features),
            nn.SiLU(),
        )


class ResidualBlock(nn.Module):
    def __init__(self, width: int) -> None:
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(width, width),
            nn.BatchNorm1d(width),
            nn.SiLU(),
            nn.Linear(width, width),
            nn.BatchNorm1d(width),
        )
        self.act = nn.SiLU()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.act(x + self.net(x))


class AeroSurrogate(nn.Module):
    """
    Aerodynamic surrogate: design_vector → [CL, CD, Cm].

    Inputs and outputs are expected to be in the *normalised* space defined
    by AeroDataset (zero-mean, unit-variance). Use dataset.encode_x_for_net()
    and dataset.denormalize_y() to convert to/from physical units.
    """

    def __init__(self, n_input: int = 10, n_output: int = 3) -> None:
        super().__init__()
        self.encoder = nn.Sequential(
            _Block(n_input, 128),
            _Block(128, 256),
        )
        self.res1 = ResidualBlock(256)
        self.res2 = ResidualBlock(256)
        self.decoder = nn.Sequential(
            _Block(256, 128),
            _Block(128, 64),
            nn.Linear(64, n_output),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        h = self.encoder(x)
        h = self.res1(h)
        h = self.res2(h)
        return self.decoder(h)

    def n_params(self) -> int:
        return sum(p.numel() for p in self.parameters() if p.requires_grad)


# ---------------------------------------------------------------------------
# Training
# ---------------------------------------------------------------------------

def _val_loss(model: AeroSurrogate, loader: DataLoader, device: torch.device) -> float:
    model.eval()
    total, count = 0.0, 0
    with torch.no_grad():
        for xb, yb in loader:
            xb, yb = xb.to(device), yb.to(device)
            pred = model(xb)
            total += F.mse_loss(pred, yb).item() * len(xb)
            count += len(xb)
    return total / max(count, 1)


def train_surrogate(
    train_subset: Subset,
    val_subset: Subset,
    checkpoint_dir: Path,
    n_epochs: int = 300,
    batch_size: int = 512,
    lr_init: float = 3e-4,
    weight_decay: float = 1e-5,
    patience: int = 30,
    device_str: str = "cpu",
    seed: int = 42,
) -> tuple[AeroSurrogate, dict[str, Any]]:
    """
    Train the aerodynamic surrogate.

    Returns (trained_model, history_dict).
    history_dict keys: "train_loss", "val_loss" (lists per epoch).
    """
    torch.manual_seed(seed)
    device = torch.device(device_str)

    checkpoint_dir = Path(checkpoint_dir)
    checkpoint_dir.mkdir(parents=True, exist_ok=True)

    train_loader = DataLoader(
        train_subset, batch_size=batch_size, shuffle=True,
        num_workers=0, pin_memory=(device.type == "cuda"),
    )
    val_loader = DataLoader(
        val_subset, batch_size=512, shuffle=False,
        num_workers=0,
    )

    model = AeroSurrogate().to(device)
    logger.info("Surrogate: %d trainable parameters", model.n_params())

    optimiser = torch.optim.AdamW(model.parameters(), lr=lr_init, weight_decay=weight_decay)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimiser, T_max=n_epochs, eta_min=1e-6
    )

    best_val = float("inf")
    best_epoch = 0
    history: dict[str, list[float]] = {"train_loss": [], "val_loss": []}

    best_ckpt = checkpoint_dir / "surrogate_best.pt"

    for epoch in range(1, n_epochs + 1):
        model.train()
        train_total, train_count = 0.0, 0

        for xb, yb in train_loader:
            xb, yb = xb.to(device), yb.to(device)
            pred = model(xb)
            loss_mse = F.mse_loss(pred, yb)
            # Physics penalty: CD (index 1) must be positive
            loss_physics = 0.1 * F.relu(-pred[:, 1]).mean()
            loss = loss_mse + loss_physics

            optimiser.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            optimiser.step()

            train_total += loss_mse.item() * len(xb)
            train_count += len(xb)

        scheduler.step()

        train_loss = train_total / max(train_count, 1)
        val_loss   = _val_loss(model, val_loader, device)

        history["train_loss"].append(train_loss)
        history["val_loss"].append(val_loss)

        if val_loss < best_val:
            best_val = val_loss
            best_epoch = epoch
            torch.save({
                "epoch": epoch,
                "model_state": model.state_dict(),
                "val_loss": val_loss,
            }, best_ckpt)

        if epoch % 20 == 0 or epoch == 1:
            logger.info(
                "Epoch %3d/%d  train=%.5f  val=%.5f  best=%.5f@%d  lr=%.2e",
                epoch, n_epochs, train_loss, val_loss, best_val, best_epoch,
                scheduler.get_last_lr()[0],
            )

        if epoch - best_epoch >= patience:
            logger.info("Early stopping at epoch %d (patience=%d)", epoch, patience)
            break

    # Restore best weights
    ckpt = torch.load(best_ckpt, map_location=device, weights_only=True)
    model.load_state_dict(ckpt["model_state"])
    logger.info("Training complete. Best val MSE=%.5f at epoch %d", best_val, best_epoch)

    return model.to("cpu"), history


def load_surrogate(checkpoint_path: Path, device_str: str = "cpu") -> AeroSurrogate:
    """Load a trained surrogate from a checkpoint file."""
    device = torch.device(device_str)
    ckpt = torch.load(checkpoint_path, map_location=device, weights_only=True)
    model = AeroSurrogate()
    if "model_state" in ckpt:
        model.load_state_dict(ckpt["model_state"])
    else:
        model.load_state_dict(ckpt)
    model.eval()
    return model


def save_surrogate(
    model: AeroSurrogate,
    path: Path,
    metadata: dict[str, Any] | None = None,
) -> None:
    """Save a trained surrogate with optional metadata."""
    torch.save({
        "model_state": model.state_dict(),
        "metadata": metadata or {},
    }, path)


# ---------------------------------------------------------------------------
# Evaluation helpers
# ---------------------------------------------------------------------------

def evaluate_r2(
    model: AeroSurrogate,
    val_subset: Subset,
    device_str: str = "cpu",
) -> dict[str, float]:
    """
    Compute R² and MAE for each output [CL, CD, Cm] on the validation set.
    Returns a dict with keys "CL_r2", "CD_r2", "Cm_r2", "CL_mae", "CD_mae", "Cm_mae".
    """
    device = torch.device(device_str)
    model.eval().to(device)

    loader = DataLoader(val_subset, batch_size=512, shuffle=False)
    preds, truths = [], []
    with torch.no_grad():
        for xb, yb in loader:
            preds.append(model(xb.to(device)).cpu())
            truths.append(yb)

    p = torch.cat(preds).numpy()
    t = torch.cat(truths).numpy()

    names = ["CL", "CD", "Cm"]
    result: dict[str, float] = {}
    for i, name in enumerate(names):
        ss_res = ((p[:, i] - t[:, i]) ** 2).sum()
        ss_tot = ((t[:, i] - t[:, i].mean()) ** 2).sum()
        r2 = 1.0 - ss_res / (ss_tot + 1e-12)
        mae = float(abs(p[:, i] - t[:, i]).mean())
        result[f"{name}_r2"] = float(r2)
        result[f"{name}_mae"] = mae

    return result
