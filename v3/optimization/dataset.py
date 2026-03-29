"""
dataset.py — PyTorch Dataset wrapping the AeroSandbox sweep NPZ.

The dataset extracts the aerodynamic outputs [CL, CD, Cm] at the operating
point closest to the target (velocity, alpha), normalises inputs and outputs
to zero-mean unit-variance, and provides a reproducible train/val split.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import Dataset, Subset


class AeroDataset(Dataset):
    """
    PyTorch Dataset over a sweep NPZ file.

    Each sample is a pair (x_norm, y_norm) where:
        x_norm : float32 tensor, shape (12,) — normalised design vector
        y_norm : float32 tensor, shape (3,)  — normalised [CL, CD, Cm]
    """

    def __init__(
        self,
        npz_path: str | Path,
        target_velocity: float = 18.0,
        target_alpha: float = 6.0,
        val_fraction: float = 0.15,
        seed: int = 42,
    ) -> None:
        npz = np.load(npz_path)

        X_raw: np.ndarray = npz["X"].astype(np.float32)           # (N, 12 or 10)
        results: np.ndarray = npz["results"].astype(np.float32)  # (N, A, V, 6)
        valid_mask: np.ndarray = npz["valid_mask"].astype(bool)  # (N,)
        alpha_range: np.ndarray = npz["alpha_range"].astype(np.float32)
        vel_range: np.ndarray = npz["vel_range"].astype(np.float32)

        # The sweep was generated with a 12-dim design space that included
        # cruise_velocity_ms (col 9) and cruise_alpha_deg (col 10).
        # The current design space is 10-dim (geometry only), mapping to
        # old columns [0,1,2,3,4,5,6,7,8, 11] (dropping 9 and 10).
        from v3.optimization.design_space import N_DIM
        if X_raw.shape[1] == 12 and N_DIM == 10:
            # Map new 10-dim space to old column indices: drop cols 9 and 10
            keep_cols = [0, 1, 2, 3, 4, 5, 6, 7, 8, 11]
            X_all = X_raw[:, keep_cols]
        elif X_raw.shape[1] == N_DIM:
            X_all = X_raw
        else:
            # Fallback: keep first N_DIM columns
            X_all = X_raw[:, :N_DIM]

        # Find operating point closest to (target_velocity, target_alpha)
        a_idx = int(np.argmin(np.abs(alpha_range - target_alpha)))
        v_idx = int(np.argmin(np.abs(vel_range - target_velocity)))

        # Extract [CL, CD, Cm] at chosen op-point, shape (N, 3)
        Y_all = results[:, a_idx, v_idx, :3].astype(np.float32)

        # Keep only valid samples with finite outputs
        finite_mask = np.isfinite(Y_all).all(axis=1)
        keep = valid_mask & finite_mask

        self._X = X_all[keep]   # (M, 12)
        self._Y = Y_all[keep]   # (M, 3)
        self._n = int(keep.sum())

        # Compute normalisation statistics
        self.x_mean = self._X.mean(axis=0)
        self.x_std  = self._X.std(axis=0) + 1e-8
        self.y_mean = self._Y.mean(axis=0)
        self.y_std  = self._Y.std(axis=0) + 1e-8

        # Normalise
        self._Xn = (self._X - self.x_mean) / self.x_std
        self._Yn = (self._Y - self.y_mean) / self.y_std

        # Reproducible train/val split
        rng = np.random.default_rng(seed)
        indices = rng.permutation(self._n)
        n_val = max(1, int(self._n * val_fraction))
        self._val_indices   = indices[:n_val].tolist()
        self._train_indices = indices[n_val:].tolist()

        self.target_velocity = target_velocity
        self.target_alpha = target_alpha
        self.n_valid = self._n

    # ------------------------------------------------------------------
    def __len__(self) -> int:
        return self._n

    def __getitem__(self, idx: int) -> tuple[torch.Tensor, torch.Tensor]:
        return (
            torch.from_numpy(self._Xn[idx]),
            torch.from_numpy(self._Yn[idx]),
        )

    # ------------------------------------------------------------------
    def get_train_val_split(self) -> tuple[Subset, Subset]:
        """Return (train_subset, val_subset) using the fixed split."""
        return Subset(self, self._train_indices), Subset(self, self._val_indices)

    # ------------------------------------------------------------------
    # Normalisation helpers (work on torch Tensors or numpy arrays)
    # ------------------------------------------------------------------

    def normalize_x(self, x: torch.Tensor) -> torch.Tensor:
        mean = torch.tensor(self.x_mean, dtype=x.dtype, device=x.device)
        std  = torch.tensor(self.x_std,  dtype=x.dtype, device=x.device)
        return (x - mean) / std

    def denormalize_x(self, xn: torch.Tensor) -> torch.Tensor:
        mean = torch.tensor(self.x_mean, dtype=xn.dtype, device=xn.device)
        std  = torch.tensor(self.x_std,  dtype=xn.dtype, device=xn.device)
        return xn * std + mean

    def normalize_y(self, y: torch.Tensor) -> torch.Tensor:
        mean = torch.tensor(self.y_mean, dtype=y.dtype, device=y.device)
        std  = torch.tensor(self.y_std,  dtype=y.dtype, device=y.device)
        return (y - mean) / std

    def denormalize_y(self, yn: torch.Tensor) -> torch.Tensor:
        mean = torch.tensor(self.y_mean, dtype=yn.dtype, device=yn.device)
        std  = torch.tensor(self.y_std,  dtype=yn.dtype, device=yn.device)
        return yn * std + mean

    # ------------------------------------------------------------------
    # Denormalize raw numpy input x ∈ [0,1]^12 through the surrogate's
    # input normalisation (sweep inputs are already in [0,1]).
    # x_norm_net = (x_raw - x_mean) / x_std  where x_raw ∈ [0,1]
    # ------------------------------------------------------------------

    def encode_x_for_net(self, x_raw: torch.Tensor) -> torch.Tensor:
        """Map raw [0,1]^12 design vector to network-input normalised space."""
        return self.normalize_x(x_raw)

    # ------------------------------------------------------------------
    # Persistence
    # ------------------------------------------------------------------

    def save_normalizer(self, path: Path) -> None:
        """Save normalisation statistics to a JSON file."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "x_mean": self.x_mean.tolist(),
            "x_std":  self.x_std.tolist(),
            "y_mean": self.y_mean.tolist(),
            "y_std":  self.y_std.tolist(),
            "target_velocity": self.target_velocity,
            "target_alpha": self.target_alpha,
            "n_valid": self.n_valid,
        }
        path.write_text(json.dumps(data, indent=2), encoding="utf-8")

    @classmethod
    def load_normalizer(cls, path: Path) -> "NormalizerOnly":
        """Load just the normalisation stats (no full dataset needed)."""
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        return NormalizerOnly(data)


class NormalizerOnly:
    """Lightweight stand-alone normaliser loaded from JSON (no dataset needed)."""

    def __init__(self, data: dict) -> None:
        self.x_mean = np.array(data["x_mean"], dtype=np.float32)
        self.x_std  = np.array(data["x_std"],  dtype=np.float32)
        self.y_mean = np.array(data["y_mean"], dtype=np.float32)
        self.y_std  = np.array(data["y_std"],  dtype=np.float32)

    def encode_x_for_net(self, x_raw: torch.Tensor) -> torch.Tensor:
        mean = torch.tensor(self.x_mean, dtype=x_raw.dtype, device=x_raw.device)
        std  = torch.tensor(self.x_std,  dtype=x_raw.dtype, device=x_raw.device)
        return (x_raw - mean) / std

    def denormalize_y(self, yn: torch.Tensor) -> torch.Tensor:
        mean = torch.tensor(self.y_mean, dtype=yn.dtype, device=yn.device)
        std  = torch.tensor(self.y_std,  dtype=yn.dtype, device=yn.device)
        return yn * std + mean
