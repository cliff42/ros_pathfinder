import math
import sys
from pathlib import Path

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPOSITORY_ROOT))

from icp_metrics_mcap import (  # noqa: E402
    ICPSample,
    sample_from_values,
    summarize,
)


def make_sample(
    status: str,
    attempted: bool,
    accepted: bool = False,
    value: float = 0.0,
    converged=None,
) -> ICPSample:
    has_result = converged is not None
    return ICPSample(
        stamp_s=value,
        status=status,
        icp_attempted=attempted,
        icp_accepted=accepted,
        current_point_count=100,
        match_count=int(value * 10) if has_result else None,
        inlier_ratio=value / 10.0 if has_result else None,
        rmse_m=value / 100.0 if has_result else None,
        iterations=int(value) if has_result else None,
        converged=converged,
        icp_processing_time_ms=value,
        correction_translation_m=value / 100.0 if has_result else None,
        correction_rotation_rad=value / 1000.0 if has_result else None,
        accepted_pose_jump_translation_m=(
            value / 100.0 if accepted else None
        ),
        accepted_pose_jump_rotation_rad=(
            value / 1000.0 if accepted else None
        ),
    )


def test_summarize_counts_attempts_rejections_and_distributions() -> None:
    samples = [
        make_sample("initialized", False),
        make_sample("stationary", False),
        make_sample("icp_accepted", True, True, 8.0, True),
        make_sample("icp_accepted", True, True, 7.0, True),
        make_sample("no_icp_result", True, value=1.0),
        make_sample("not_converged", True, value=6.0, converged=False),
        make_sample("high_rmse", True, value=5.0, converged=True),
        make_sample("low_inlier_ratio", True, value=2.0, converged=True),
        make_sample("odom_disagreement", True, value=4.0, converged=True),
    ]

    metrics = summarize(samples)

    assert metrics["processed_scans"] == 9
    assert metrics["icp_attempts"] == 7
    assert metrics["icp_accepted"] == 2
    assert metrics["acceptance_rate_pct"] == pytest.approx(200.0 / 7.0)
    for status in (
        "no_icp_result",
        "not_converged",
        "high_rmse",
        "low_inlier_ratio",
        "odom_disagreement",
    ):
        assert metrics[f"reject_{status}_count"] == 1
        assert metrics[f"reject_{status}_pct_of_attempts"] == pytest.approx(
            100.0 / 7.0
        )
    assert metrics["inlier_ratio_p5"] == pytest.approx(0.25)
    assert metrics["inlier_ratio_p50"] == pytest.approx(0.55)
    assert metrics["rmse_m_p95"] == pytest.approx(0.0775)
    assert metrics["convergence_rate_of_attempts_pct"] == pytest.approx(
        500.0 / 7.0
    )
    assert metrics["convergence_rate_of_results_pct"] == pytest.approx(
        500.0 / 6.0
    )
    assert metrics["max_accepted_pose_jump_translation_m"] == 0.08
    assert metrics["max_accepted_pose_jump_rotation_rad"] == 0.008


def test_sample_parser_uses_correction_as_accepted_jump_fallback() -> None:
    sample = sample_from_values(
        stamp_s=12.5,
        message="icp_accepted",
        values={
            "schema_version": "1",
            "status": "icp_accepted",
            "icp_attempted": "true",
            "icp_accepted": "true",
            "current_point_count": "100",
            "match_count": "80",
            "inlier_ratio": "0.8",
            "rmse_m": "0.025",
            "iterations": "4",
            "converged": "true",
            "icp_processing_time_ms": "3.2",
            "correction_translation_m": "0.012",
            "correction_rotation_rad": "0.03",
        },
    )

    assert sample.icp_attempted
    assert sample.icp_accepted
    assert sample.inlier_ratio == 0.8
    assert sample.accepted_pose_jump_translation_m == 0.012
    assert sample.accepted_pose_jump_rotation_rad == 0.03


def test_empty_attempt_set_returns_nan_rates() -> None:
    metrics = summarize([make_sample("stationary", False)])

    assert metrics["icp_attempts"] == 0
    assert math.isnan(float(metrics["acceptance_rate_pct"]))
    assert math.isnan(float(metrics["inlier_ratio_p5"]))
