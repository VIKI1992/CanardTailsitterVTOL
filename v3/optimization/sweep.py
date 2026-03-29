"""
sweep.py — Parallel AeroSandbox design-space sweep for surrogate training data.

Usage:
    from v3.optimization.sweep import run_sweep

The sweep evaluates each LHS-sampled design at a grid of (alpha, velocity)
operating points and saves results to an NPZ file.  Failed evaluations are
masked out rather than dropped so array shapes remain consistent.

Resumability: a pickle checkpoint is written after each completed batch so
interrupted runs can continue from where they left off.
"""
from __future__ import annotations

import concurrent.futures
import logging
import os
import pickle
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

from v3.optimization.design_space import sample_lhs, to_design_parameters

logger = logging.getLogger(__name__)

# Default operating-point grids
DEFAULT_ALPHA_RANGE = np.linspace(-4.0, 16.0, 9)   # degrees
DEFAULT_VEL_RANGE   = np.linspace(14.0, 26.0, 5)   # m/s


# ---------------------------------------------------------------------------
# Worker function — must be module-level for multiprocessing pickle
# ---------------------------------------------------------------------------

def _evaluate_one(
    args: tuple[int, np.ndarray, np.ndarray, np.ndarray, str, str],
) -> tuple[int, np.ndarray | None]:
    """
    Evaluate one design sample across the full (alpha, velocity) grid.

    Returns (sample_index, results_array) where results_array has shape
    (n_alpha, n_vel, 6) with columns [CL, CD, Cm, CY, Cl, Cn], or None on
    catastrophic failure.
    """
    idx, x_norm, alpha_range, vel_range, params_path, repo_root_str = args

    # Import inside worker to avoid multiprocessing serialisation issues
    import sys
    repo_root = Path(repo_root_str)
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))

    from v3.model.analysis import run_aero
    from v3.model.geometry import derive_geometry
    from v3.model.spec import load_parameters

    base_params = load_parameters(Path(params_path))

    try:
        params = to_design_parameters(x_norm, base_params)
        derived = derive_geometry(params)
    except Exception:
        return idx, None

    n_a = len(alpha_range)
    n_v = len(vel_range)
    result = np.full((n_a, n_v, 6), np.nan, dtype=np.float32)

    for ia, alpha in enumerate(alpha_range):
        for iv, vel in enumerate(vel_range):
            try:
                snap = run_aero(
                    params, derived, repo_root,
                    velocity_m_s=float(vel),
                    alpha_deg=float(alpha),
                )
                result[ia, iv] = [snap.CL, snap.CD, snap.Cm, snap.CY, snap.Cl, snap.Cn]
            except Exception:
                pass  # leaves NaN in result

    # Mark as valid only if cruise-like conditions produced non-NaN CL/CD
    mid_a = n_a // 2
    mid_v = n_v // 2
    if np.isnan(result[mid_a, mid_v, 0]) or np.isnan(result[mid_a, mid_v, 1]):
        return idx, None

    return idx, result


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def run_sweep(
    params_path: Path,
    repo_root: Path,
    n_samples: int = 2000,
    alpha_range: np.ndarray | None = None,
    vel_range: np.ndarray | None = None,
    n_workers: int | None = None,
    output_dir: Path | None = None,
    seed: int = 42,
    batch_size: int = 100,
) -> Path:
    """
    Run a Latin-Hypercube sweep over the 12-dimensional design space.

    Args:
        params_path:   Path to parameters_v3.json (baseline design).
        repo_root:     Root of the CanardTailsitterVTOL repo (for airfoil paths).
        n_samples:     Number of LHS samples to evaluate.
        alpha_range:   Array of angle-of-attack values in degrees.
        vel_range:     Array of velocities in m/s.
        n_workers:     Number of parallel worker processes (default: cpu_count-1).
        output_dir:    Directory for the output NPZ and progress checkpoint.
        seed:          RNG seed for LHS.
        batch_size:    Samples per checkpoint batch.

    Returns:
        Path to the saved NPZ file.
    """
    if alpha_range is None:
        alpha_range = DEFAULT_ALPHA_RANGE
    if vel_range is None:
        vel_range = DEFAULT_VEL_RANGE
    if n_workers is None:
        n_workers = max(1, (os.cpu_count() or 2) - 1)
    if output_dir is None:
        output_dir = repo_root / "data" / "sweep"
    output_dir.mkdir(parents=True, exist_ok=True)

    n_a = len(alpha_range)
    n_v = len(vel_range)

    # Sample the full design space upfront
    X = sample_lhs(n_samples, seed=seed)  # (n_samples, 12)

    # Checkpoint file for resumability
    ckpt_path = output_dir / "sweep_progress.pkl"
    if ckpt_path.exists():
        with open(ckpt_path, "rb") as f:
            checkpoint = pickle.load(f)
        if (
            checkpoint.get("n_samples") == n_samples
            and checkpoint.get("seed") == seed
            and checkpoint.get("n_a") == n_a
            and checkpoint.get("n_v") == n_v
        ):
            results_arr = checkpoint["results_arr"]
            valid_mask = checkpoint["valid_mask"]
            completed = set(checkpoint["completed"])
            logger.info("Resuming sweep from checkpoint (%d/%d done)", len(completed), n_samples)
        else:
            logger.info("Checkpoint mismatch — starting fresh sweep")
            results_arr = np.full((n_samples, n_a, n_v, 6), np.nan, dtype=np.float32)
            valid_mask = np.zeros(n_samples, dtype=bool)
            completed: set[int] = set()
    else:
        results_arr = np.full((n_samples, n_a, n_v, 6), np.nan, dtype=np.float32)
        valid_mask = np.zeros(n_samples, dtype=bool)
        completed: set[int] = set()

    pending = [i for i in range(n_samples) if i not in completed]
    total = len(pending)
    logger.info("Sweep: %d samples × %d α × %d v = %d AeroSandbox calls",
                total, n_a, n_v, total * n_a * n_v)
    logger.info("Using %d worker processes", n_workers)

    params_str = str(params_path)
    repo_str = str(repo_root)

    done_count = 0
    for batch_start in range(0, len(pending), batch_size):
        batch_indices = pending[batch_start: batch_start + batch_size]
        args_list = [
            (i, X[i], alpha_range, vel_range, params_str, repo_str)
            for i in batch_indices
        ]

        with concurrent.futures.ProcessPoolExecutor(max_workers=n_workers) as executor:
            futures = {executor.submit(_evaluate_one, a): a[0] for a in args_list}
            for future in concurrent.futures.as_completed(futures):
                idx, res = future.result()
                if res is not None:
                    results_arr[idx] = res
                    valid_mask[idx] = True
                completed.add(idx)
                done_count += 1
                if done_count % 50 == 0:
                    pct = 100.0 * (len(completed)) / n_samples
                    valid_pct = 100.0 * valid_mask.sum() / max(1, len(completed))
                    logger.info("  %d/%d (%.0f%%) — valid rate %.1f%%",
                                len(completed), n_samples, pct, valid_pct)

        # Save checkpoint after each batch
        with open(ckpt_path, "wb") as f:
            pickle.dump({
                "n_samples": n_samples,
                "seed": seed,
                "n_a": n_a,
                "n_v": n_v,
                "results_arr": results_arr,
                "valid_mask": valid_mask,
                "completed": list(completed),
            }, f)

    n_valid = valid_mask.sum()
    logger.info("Sweep complete: %d/%d valid (%.1f%%)", n_valid, n_samples,
                100.0 * n_valid / n_samples)

    # Save final NPZ
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    npz_path = output_dir / f"sweep_{timestamp}.npz"
    np.savez_compressed(
        npz_path,
        X=X.astype(np.float32),
        results=results_arr,      # (N, A, V, 6): [CL, CD, Cm, CY, Cl, Cn]
        valid_mask=valid_mask,
        alpha_range=alpha_range.astype(np.float32),
        vel_range=vel_range.astype(np.float32),
        n_samples=np.array([n_samples]),
        seed=np.array([seed]),
    )
    logger.info("Saved sweep to %s", npz_path)

    # Clean up checkpoint
    if ckpt_path.exists():
        ckpt_path.unlink()

    return npz_path


def find_latest_sweep(sweep_dir: Path) -> Path | None:
    """Return the most recently modified sweep NPZ in the given directory."""
    npz_files = sorted(sweep_dir.glob("sweep_*.npz"), key=lambda p: p.stat().st_mtime)
    return npz_files[-1] if npz_files else None
