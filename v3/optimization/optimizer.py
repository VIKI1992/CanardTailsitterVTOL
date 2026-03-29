"""
optimizer.py — Hybrid Differential Evolution + gradient-descent design optimizer.

Two-phase strategy:
  Phase 1: Differential Evolution over [0,1]^12 through the surrogate (fast global search).
  Phase 2: Multi-start Adam gradient descent through the differentiable surrogate
           (exploits neural-net differentiability for rapid local convergence).

Total wall time: ~25 seconds for 200 restarts vs ~3 hours for DE over raw AeroSandbox.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np
import torch
from scipy.optimize import differential_evolution

from v3.optimization.design_space import (
    N_DIM,
    clamp,
    decode,
    get_bounds_normalised,
    sample_lhs,
    x_to_dict,
)
from v3.optimization.objectives import surrogate_objective

if TYPE_CHECKING:
    from v3.optimization.dataset import AeroDataset
    from v3.optimization.surrogate import AeroSurrogate

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Result dataclass
# ---------------------------------------------------------------------------

@dataclass
class OptimumCandidate:
    rank: int
    x_norm: np.ndarray                    # (12,) normalised [0,1]
    x_raw: dict[str, float]               # decoded raw values
    surrogate_ld: float                   # surrogate-predicted L/D
    surrogate_obj: float                  # full penalty_objective value
    # Filled after AeroSandbox verification:
    aero_verified: bool = False
    aero_ld: float = float("nan")
    aero_cl: float = float("nan")
    aero_cd: float = float("nan")
    aero_cm: float = float("nan")
    n_violations: int = -1
    violation_details: list[str] = field(default_factory=list)

    @property
    def delta_vs_baseline_pct(self) -> float:
        return float("nan")  # computed externally in nn_optimize.py


# ---------------------------------------------------------------------------
# Core optimizer class
# ---------------------------------------------------------------------------

class SurrogateOptimizer:
    """
    Hybrid Differential Evolution + multi-start gradient-descent optimizer.

    The surrogate forward pass costs ~0.1 ms; gradient-based Adam steps cost
    ~0.05 ms each.  The entire optimization runs in under a minute on CPU.
    """

    def __init__(
        self,
        surrogate: "AeroSurrogate",
        dataset: "AeroDataset",
        n_restarts: int = 200,
        gd_steps: int = 500,
        gd_lr: float = 0.02,
        de_popsize: int = 6,          # population = popsize * N_DIM = 72
        de_maxiter: int = 1000,
        seed: int = 42,
    ) -> None:
        self.surrogate = surrogate.eval()
        self.dataset   = dataset
        self.n_restarts = n_restarts
        self.gd_steps   = gd_steps
        self.gd_lr      = gd_lr
        self.de_popsize = de_popsize
        self.de_maxiter = de_maxiter
        self.seed       = seed

    # ------------------------------------------------------------------
    # Surrogate fitness (numpy interface for scipy)
    # ------------------------------------------------------------------

    def _fitness(self, x: np.ndarray) -> float:
        x_t = torch.tensor(x, dtype=torch.float32)
        with torch.no_grad():
            return float(surrogate_objective(x_t, self.surrogate, self.dataset))

    def _surrogate_ld(self, x: np.ndarray) -> float:
        x_t = torch.tensor(x, dtype=torch.float32)
        with torch.no_grad():
            x_net = self.dataset.encode_x_for_net(x_t.unsqueeze(0))
            y_norm = self.surrogate(x_net).squeeze(0)
            y_phys = self.dataset.denormalize_y(y_norm)
            cl, cd = float(y_phys[0]), float(y_phys[1])
        return cl / (cd + 1e-6)

    # ------------------------------------------------------------------
    # Phase 1: Differential Evolution
    # ------------------------------------------------------------------

    def _run_de(self) -> np.ndarray:
        """Return best x from DE over the surrogate landscape."""
        logger.info("Phase 1: Differential Evolution (popsize=%d × %d, maxiter=%d)...",
                    self.de_popsize, N_DIM, self.de_maxiter)
        result = differential_evolution(
            self._fitness,
            bounds=get_bounds_normalised(),
            strategy="best1bin",
            maxiter=self.de_maxiter,
            popsize=self.de_popsize,
            mutation=(0.5, 1.5),
            recombination=0.9,
            tol=1e-7,
            seed=self.seed,
            polish=False,
            workers=1,
        )
        logger.info("  DE converged: obj=%.5f  L/D≈%.2f",
                    result.fun, self._surrogate_ld(result.x))
        return result.x.copy()

    # ------------------------------------------------------------------
    # Phase 2: Single gradient-descent run from x0
    # ------------------------------------------------------------------

    def _run_gd(self, x0: np.ndarray) -> tuple[np.ndarray, float]:
        """
        Run Adam gradient descent from x0.
        Returns (best_x, best_obj) after gd_steps steps.
        """
        x = torch.tensor(x0, dtype=torch.float32, requires_grad=False)
        x = x.detach().clone().requires_grad_(True)
        opt = torch.optim.Adam([x], lr=self.gd_lr)

        best_x   = x.detach().clone().numpy()
        best_obj = float("inf")

        for _ in range(self.gd_steps):
            opt.zero_grad()
            loss = surrogate_objective(x, self.surrogate, self.dataset)
            loss.backward()
            opt.step()

            with torch.no_grad():
                x.clamp_(0.0, 1.0)

            obj_val = loss.item()
            if obj_val < best_obj:
                best_obj = obj_val
                best_x   = x.detach().clone().numpy()

        return best_x, best_obj

    # ------------------------------------------------------------------
    # Full hybrid optimization
    # ------------------------------------------------------------------

    def optimize(self, top_k: int = 20) -> list[OptimumCandidate]:
        """
        Run Phase 1 (DE) then Phase 2 (multi-start GD).
        Returns the top_k candidates ranked by surrogate_obj (lowest = best).
        """
        # Phase 1: DE global search
        de_best = self._run_de()

        # Phase 2: multi-start GD
        lhs_seeds = sample_lhs(self.n_restarts - 1, seed=self.seed + 1)
        starts = np.vstack([de_best[None, :], lhs_seeds])  # (n_restarts, 12)

        logger.info("Phase 2: %d gradient-descent restarts (%d steps each)...",
                    self.n_restarts, self.gd_steps)

        candidates: list[tuple[np.ndarray, float]] = []
        for i, x0 in enumerate(starts):
            best_x, best_obj = self._run_gd(x0)
            best_x = clamp(best_x)
            candidates.append((best_x, best_obj))
            if (i + 1) % 50 == 0:
                logger.info("  GD %d/%d  best_obj_so_far=%.5f",
                            i + 1, self.n_restarts,
                            min(c[1] for c in candidates))

        # Sort by objective
        candidates.sort(key=lambda c: c[1])

        # Build result objects (deduplicate by proximity)
        results: list[OptimumCandidate] = []
        seen: list[np.ndarray] = []
        rank = 0

        for x, obj in candidates:
            # Simple deduplication: skip if very close to an already-selected candidate
            too_close = any(np.linalg.norm(x - s) < 0.02 for s in seen)
            if too_close:
                continue
            seen.append(x)
            rank += 1
            results.append(OptimumCandidate(
                rank=rank,
                x_norm=x.copy(),
                x_raw=x_to_dict(x),
                surrogate_ld=self._surrogate_ld(x),
                surrogate_obj=obj,
            ))
            if len(results) >= top_k:
                break

        logger.info("Found %d distinct candidates.  Best surrogate L/D = %.2f",
                    len(results), results[0].surrogate_ld if results else 0.0)
        return results
