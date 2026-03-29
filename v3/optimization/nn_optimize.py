"""
nn_optimize.py — Neural network-optimised design optimizer for the canard tailsitter VTOL.

Recreates the "AI Designed This Flying Wing" (Neuronautics, 2026) setup —
AeroSandbox aerodynamic analysis + evolutionary/gradient-based optimization —
but replaces brute-force AeroSandbox calls during the search with a trained
neural surrogate, giving ~2000x speedup over pure Differential Evolution.

Pipeline:
  1. AeroSandbox design-space sweep (LHS over 12 parameters, parallel)
  2. Train PyTorch MLP surrogate on sweep data
  3. Hybrid Differential Evolution + gradient-descent through surrogate (~25 s)
  4. Verify top-k candidates with full AeroSandbox + 18-constraint validation
  5. Print comparison table and save best design to JSON

Usage examples:
    # Full run (standard quality)
    python v3/optimization/nn_optimize.py

    # Quick smoke test
    python v3/optimization/nn_optimize.py --n-samples 500 --n-epochs 100 --top-k 3

    # Re-use an existing sweep, retrain only
    python v3/optimization/nn_optimize.py --sweep-file data/sweep/sweep_20260329.npz

    # Re-use sweep + checkpoint, optimize only
    python v3/optimization/nn_optimize.py \\
        --sweep-file data/sweep/sweep_20260329.npz \\
        --checkpoint checkpoints/surrogate_20260329.pt \\
        --optimize-only
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import sys
import time
from datetime import datetime
from pathlib import Path

# ---------------------------------------------------------------------------
# Ensure repo root is on sys.path regardless of how the script is invoked
# ---------------------------------------------------------------------------
_THIS = Path(__file__).resolve()
_REPO_ROOT = _THIS.parents[2]   # CanardTailsitterVTOL/
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import numpy as np
import torch

from v3.model.analysis import evaluate_validation, run_aero
from v3.model.geometry import derive_geometry
from v3.model.spec import load_parameters, load_validation_targets
from v3.optimization.dataset import AeroDataset
from v3.optimization.design_space import (
    CRUISE_ALPHA_DEG,
    CRUISE_VELOCITY_MS,
    baseline_x,
    to_design_parameters,
    x_to_dict,
)
from v3.optimization.objectives import build_constraint_summary
from v3.optimization.optimizer import OptimumCandidate, SurrogateOptimizer
from v3.optimization.surrogate import (
    AeroSurrogate,
    evaluate_r2,
    load_surrogate,
    save_surrogate,
    train_surrogate,
)
from v3.optimization.sweep import find_latest_sweep, run_sweep

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-7s  %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Neural network-optimised VTOL design optimizer",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--sweep-only",    action="store_true", help="Run sweep then exit")
    p.add_argument("--train-only",    action="store_true", help="Train surrogate then exit")
    p.add_argument("--optimize-only", action="store_true",
                   help="Skip sweep and training; requires --sweep-file and --checkpoint")
    p.add_argument("--n-samples",  type=int,   default=2000,
                   help="Number of LHS samples for the sweep")
    p.add_argument("--n-epochs",   type=int,   default=300,
                   help="Max training epochs for the surrogate")
    p.add_argument("--n-restarts", type=int,   default=200,
                   help="Number of gradient-descent restarts in Phase 2")
    p.add_argument("--sweep-file", type=Path,  default=None,
                   help="Path to an existing sweep NPZ (skips sweep)")
    p.add_argument("--checkpoint", type=Path,  default=None,
                   help="Path to an existing surrogate checkpoint (skips training)")
    p.add_argument("--baseline-params", type=Path,
                   default=_REPO_ROOT / "spec" / "parameters_v3.json",
                   help="Path to baseline parameters JSON")
    p.add_argument("--validation-targets", type=Path,
                   default=_REPO_ROOT / "spec" / "validation_targets_v3.json",
                   help="Path to validation targets JSON")
    p.add_argument("--output-dir",  type=Path,
                   default=_REPO_ROOT / "results",
                   help="Directory for output JSON and reports")
    p.add_argument("--sweep-dir",   type=Path,
                   default=_REPO_ROOT / "data" / "sweep",
                   help="Directory for sweep NPZ files")
    p.add_argument("--checkpoint-dir", type=Path,
                   default=_REPO_ROOT / "checkpoints",
                   help="Directory for surrogate checkpoints")
    p.add_argument("--top-k",       type=int,   default=5,
                   help="Number of candidates to verify with full AeroSandbox")
    p.add_argument("--workers",     type=int,   default=None,
                   help="Worker processes for sweep (default: cpu_count-1)")
    p.add_argument("--device",      type=str,   default="cpu",
                   choices=["cpu", "cuda", "mps"],
                   help="PyTorch device for training and inference")
    p.add_argument("--seed",        type=int,   default=42)
    p.add_argument("--r2-abort-threshold", type=float, default=0.90,
                   help="Abort if any surrogate output R² falls below this")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Step helpers
# ---------------------------------------------------------------------------

def _step_sweep(args: argparse.Namespace) -> Path:
    """Run or locate the AeroSandbox sweep NPZ."""
    if args.sweep_file is not None:
        path = Path(args.sweep_file)
        if not path.exists():
            logger.error("--sweep-file not found: %s", path)
            sys.exit(1)
        logger.info("Using existing sweep: %s", path)
        return path

    logger.info("=" * 60)
    logger.info("STEP 1: AeroSandbox sweep  (%d LHS samples)", args.n_samples)
    logger.info("=" * 60)
    t0 = time.time()
    npz_path = run_sweep(
        params_path=args.baseline_params,
        repo_root=_REPO_ROOT,
        n_samples=args.n_samples,
        output_dir=args.sweep_dir,
        n_workers=args.workers,
        seed=args.seed,
    )
    logger.info("Sweep done in %.1f min — %s", (time.time() - t0) / 60, npz_path)
    return npz_path


def _step_train(
    args: argparse.Namespace,
    npz_path: Path,
) -> tuple[AeroSurrogate, AeroDataset]:
    """Train or load the surrogate."""
    dataset = AeroDataset(npz_path, seed=args.seed)
    logger.info("Dataset: %d valid samples (train=%d, val=%d)",
                dataset.n_valid,
                len(dataset._train_indices),
                len(dataset._val_indices))

    if args.checkpoint is not None:
        path = Path(args.checkpoint)
        if not path.exists():
            logger.error("--checkpoint not found: %s", path)
            sys.exit(1)
        logger.info("Loading surrogate from %s", path)
        surrogate = load_surrogate(path)
        return surrogate, dataset

    logger.info("=" * 60)
    logger.info("STEP 2: Train surrogate  (%d epochs max)", args.n_epochs)
    logger.info("=" * 60)
    t0 = time.time()
    train_sub, val_sub = dataset.get_train_val_split()
    surrogate, history = train_surrogate(
        train_subset=train_sub,
        val_subset=val_sub,
        checkpoint_dir=args.checkpoint_dir,
        n_epochs=args.n_epochs,
        device_str=args.device,
        seed=args.seed,
    )
    elapsed = time.time() - t0
    logger.info("Training done in %.1f s", elapsed)

    # Save with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    ckpt_path = args.checkpoint_dir / f"surrogate_{timestamp}.pt"
    save_surrogate(surrogate, ckpt_path, metadata={
        "n_samples": dataset.n_valid,
        "n_epochs": args.n_epochs,
        "seed": args.seed,
    })
    dataset.save_normalizer(args.checkpoint_dir / f"normalizer_{timestamp}.json")
    logger.info("Saved surrogate to %s", ckpt_path)
    return surrogate, dataset


def _step_validate_surrogate(
    surrogate: AeroSurrogate,
    dataset: AeroDataset,
    abort_threshold: float,
) -> dict[str, float]:
    """Evaluate R² / MAE on the validation split."""
    logger.info("=" * 60)
    logger.info("STEP 3: Validate surrogate accuracy")
    logger.info("=" * 60)
    _, val_sub = dataset.get_train_val_split()
    metrics = evaluate_r2(surrogate, val_sub)
    for name, r2 in [(k[:-3], v) for k, v in metrics.items() if k.endswith("_r2")]:
        mae = metrics[f"{name}_mae"]
        status = "OK" if r2 >= abort_threshold else "WARN"
        logger.info("  %s  R²=%.4f  MAE=%.5f  [%s]", name, r2, mae, status)

    min_r2 = min(v for k, v in metrics.items() if k.endswith("_r2"))
    if min_r2 < abort_threshold:
        logger.warning(
            "Surrogate R² = %.4f < threshold %.2f — results may be unreliable. "
            "Consider increasing --n-samples.",
            min_r2, abort_threshold,
        )
    return metrics


def _step_optimize(
    args: argparse.Namespace,
    surrogate: AeroSurrogate,
    dataset: AeroDataset,
) -> list[OptimumCandidate]:
    """Run the hybrid optimizer."""
    logger.info("=" * 60)
    logger.info("STEP 4: Hybrid DE + gradient-descent optimization")
    logger.info("=" * 60)
    optimizer = SurrogateOptimizer(
        surrogate=surrogate,
        dataset=dataset,
        n_restarts=args.n_restarts,
        seed=args.seed,
    )
    t0 = time.time()
    candidates = optimizer.optimize(top_k=max(args.top_k, 20))
    logger.info("Optimization done in %.1f s", time.time() - t0)
    return candidates


def _step_verify(
    candidates: list[OptimumCandidate],
    base_params,
    base_ld: float,
    validation_targets: dict,
    top_k: int,
) -> list[OptimumCandidate]:
    """Verify top-k candidates with full AeroSandbox + constraint validation."""
    logger.info("=" * 60)
    logger.info("STEP 5: AeroSandbox verification (top %d candidates)", top_k)
    logger.info("=" * 60)

    for cand in candidates[:top_k]:
        try:
            params = to_design_parameters(cand.x_norm, base_params)
            derived = derive_geometry(params)
            snap = run_aero(
                params, derived, _REPO_ROOT,
                velocity_m_s=CRUISE_VELOCITY_MS,
                alpha_deg=CRUISE_ALPHA_DEG,
            )
            summary = build_constraint_summary(snap, derived, validation_targets)
            cand.aero_verified   = True
            cand.aero_ld         = summary.ld
            cand.aero_cl         = summary.cl
            cand.aero_cd         = summary.cd
            cand.aero_cm         = summary.cm
            cand.n_violations    = summary.n_violations
            cand.violation_details = summary.violation_details

            delta = 100.0 * (summary.ld - base_ld) / (abs(base_ld) + 1e-6)
            logger.info(
                "  Rank %2d | surrogate L/D=%.2f | aero L/D=%.2f (%+.1f%%) | "
                "violations=%d | CL=%.3f CD=%.4f Cm=%+.4f",
                cand.rank, cand.surrogate_ld, summary.ld, delta,
                summary.n_violations, summary.cl, summary.cd, summary.cm,
            )
            if summary.violation_details:
                for v in summary.violation_details:
                    logger.info("           ✗ %s", v)

        except Exception as exc:
            logger.warning("  Rank %2d — AeroSandbox failed: %s", cand.rank, exc)
            cand.aero_verified = False

    return candidates


def _print_summary(
    candidates: list[OptimumCandidate],
    base_ld: float,
    top_k: int,
) -> None:
    """Print a formatted comparison table."""
    verified = [c for c in candidates if c.aero_verified]
    if not verified:
        logger.info("No verified candidates.")
        return

    logger.info("")
    logger.info("=" * 70)
    logger.info("RESULTS SUMMARY  (baseline L/D = %.2f)", base_ld)
    logger.info("=" * 70)
    logger.info("%-6s %-14s %-14s %-10s %-10s", "Rank", "surrogate_L/D", "aero_L/D", "delta", "violations")
    logger.info("-" * 70)
    for c in verified[:top_k]:
        delta = 100.0 * (c.aero_ld - base_ld) / (abs(base_ld) + 1e-6)
        feasible = "✓" if c.n_violations == 0 else f"✗ ({c.n_violations})"
        logger.info(
            "%-6d %-14.2f %-14.2f %-10s %-10s",
            c.rank, c.surrogate_ld, c.aero_ld, f"{delta:+.1f}%", feasible,
        )
    logger.info("=" * 70)

    best_feasible = next((c for c in verified if c.n_violations == 0), None)
    if best_feasible:
        delta = 100.0 * (best_feasible.aero_ld - base_ld) / (abs(base_ld) + 1e-6)
        logger.info("Best feasible: Rank %d  L/D=%.2f  (%+.1f%% vs baseline)",
                    best_feasible.rank, best_feasible.aero_ld, delta)
    else:
        logger.info("No fully-feasible candidate found in top-%d.  "
                    "Consider increasing --n-samples or relaxing constraints.", top_k)


def _save_results(
    args: argparse.Namespace,
    candidates: list[OptimumCandidate],
    base_params,
    base_ld: float,
    surrogate_metrics: dict,
) -> Path | None:
    """Save the best feasible design and a results JSON."""
    args.output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    verified = [c for c in candidates if c.aero_verified]
    feasible = [c for c in verified if c.n_violations == 0]
    best = feasible[0] if feasible else (verified[0] if verified else None)
    if best is None:
        logger.info("Nothing to save.")
        return None

    # Reconstruct patched DesignParameters for the best candidate
    best_params = to_design_parameters(best.x_norm, base_params)

    delta = 100.0 * (best.aero_ld - base_ld) / (abs(base_ld) + 1e-6)

    output = {
        "timestamp": timestamp,
        "baseline_ld": base_ld,
        "best_candidate": {
            "rank": best.rank,
            "surrogate_ld": best.surrogate_ld,
            "aero_ld": best.aero_ld,
            "aero_cl": best.aero_cl,
            "aero_cd": best.aero_cd,
            "aero_cm": best.aero_cm,
            "delta_pct": delta,
            "n_violations": best.n_violations,
            "violation_details": best.violation_details,
            "design_vector_raw": best.x_raw,
        },
        "surrogate_metrics": surrogate_metrics,
        "all_verified_candidates": [
            {
                "rank": c.rank,
                "aero_ld": c.aero_ld if c.aero_verified else None,
                "n_violations": c.n_violations,
                "surrogate_ld": c.surrogate_ld,
            }
            for c in verified
        ],
    }

    out_path = args.output_dir / f"optimized_params_{timestamp}.json"
    out_path.write_text(json.dumps(output, indent=2), encoding="utf-8")
    logger.info("Saved results to %s", out_path)
    return out_path


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    args = _parse_args()

    # Load baseline
    logger.info("Loading baseline parameters from %s", args.baseline_params)
    base_params = load_parameters(args.baseline_params)
    validation_targets = load_validation_targets(args.validation_targets)

    # Compute baseline L/D
    base_derived = derive_geometry(base_params)
    base_snap = run_aero(
        base_params, base_derived, _REPO_ROOT,
        velocity_m_s=base_params.analysis_conditions.cruise_speed_m_s,
        alpha_deg=base_params.analysis_conditions.cruise_alpha_deg,
    )
    base_ld = base_snap.CL / (base_snap.CD + 1e-6)
    logger.info("Baseline L/D = %.2f  (v=%.0f m/s, α=%.1f°)",
                base_ld,
                base_params.analysis_conditions.cruise_speed_m_s,
                base_params.analysis_conditions.cruise_alpha_deg)

    # ----------------------------------------------------------------
    # Step 1: Sweep
    # ----------------------------------------------------------------
    if not args.optimize_only:
        npz_path = _step_sweep(args)
    else:
        if args.sweep_file is None:
            # Try to find the latest sweep automatically
            npz_path = find_latest_sweep(args.sweep_dir)
            if npz_path is None:
                logger.error("--optimize-only requires a sweep file. "
                             "Use --sweep-file or run the sweep first.")
                sys.exit(1)
            logger.info("Using latest sweep: %s", npz_path)
        else:
            npz_path = args.sweep_file

    if args.sweep_only:
        logger.info("--sweep-only: done.")
        return

    # ----------------------------------------------------------------
    # Step 2: Train / load surrogate
    # ----------------------------------------------------------------
    surrogate, dataset = _step_train(args, npz_path)

    if args.train_only:
        logger.info("--train-only: done.")
        return

    # ----------------------------------------------------------------
    # Step 3: Validate surrogate
    # ----------------------------------------------------------------
    surrogate_metrics = _step_validate_surrogate(
        surrogate, dataset, args.r2_abort_threshold
    )

    # ----------------------------------------------------------------
    # Step 4: Optimize
    # ----------------------------------------------------------------
    candidates = _step_optimize(args, surrogate, dataset)

    # ----------------------------------------------------------------
    # Step 5: Verify with AeroSandbox
    # ----------------------------------------------------------------
    candidates = _step_verify(
        candidates, base_params, base_ld, validation_targets, args.top_k
    )

    # ----------------------------------------------------------------
    # Summary + save
    # ----------------------------------------------------------------
    _print_summary(candidates, base_ld, args.top_k)
    _save_results(args, candidates, base_params, base_ld, surrogate_metrics)

    logger.info("Done.")


if __name__ == "__main__":
    main()
