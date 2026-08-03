# safety.py — Safety checker evaluating telemetry against configured limits.
# Returns a typed SafetyResult on every check — never silently passes.

from __future__ import annotations

from dataclasses import dataclass
import logging

from telemetry import TelemetryData

import config

logger = logging.getLogger("SAFETY")

@dataclass
class SafetyResult:
    """Result of a single safety evaluation."""

    ok: bool
    reason: str = "OK"

class SafetyChecker:
    """
    Evaluates telemetry against configured safety limits.
    Returns the first failing check with a descriptive reason string.
    """

    def check(self, telemetry: TelemetryData) -> SafetyResult:
        """
        Run all safety checks against the current telemetry snapshot.

        Returns SafetyResult(ok=True) if all checks pass, otherwise
        returns the first failing check.
        """
        # 1. Telemetry staleness
        if telemetry.age_s > config.MAX_TELEMETRY_AGE_S:
            return SafetyResult(
                ok=False,
                reason=(
                    f"Telemetry stale: age {telemetry.age_s:.2f}s "
                    f"> max {config.MAX_TELEMETRY_AGE_S:.2f}s"
                ),
            )

        # 2. Minimum relative altitude
        if telemetry.rel_alt_m < config.MIN_REL_ALT_M:
            return SafetyResult(
                ok=False,
                reason=(
                    f"Altitude too low: {telemetry.rel_alt_m:.1f}m "
                    f"< min {config.MIN_REL_ALT_M:.1f}m"
                ),
            )

        # 3. Maximum groundspeed
        if telemetry.groundspeed_m_s > config.MAX_GROUNDSPEED_M_S:
            return SafetyResult(
                ok=False,
                reason=(
                    f"Overspeed: {telemetry.groundspeed_m_s:.1f}m/s "
                    f"> max {config.MAX_GROUNDSPEED_M_S:.1f}m/s"
                ),
            )

        return SafetyResult(ok=True)