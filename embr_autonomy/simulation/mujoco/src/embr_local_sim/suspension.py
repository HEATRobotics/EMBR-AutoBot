"""Vertical travel for one roller. Assumed. The STEP has no suspension joint.

MuJoCo has no belt, so a track shoe cannot climb by itself. A slide along
body +Z lets one roller rise when a rock pushes it, while the other three
stay down. The range and the spring are not a measured suspension.
"""

from __future__ import annotations

# ASSUMED. Enough for a rock under one idler, not a measured stroke.
SLIDE_RANGE_M = 0.08
# Soft enough that a rock lifts one roller, stiff enough that the chassis
# does not drop onto the plate under its own weight. Both numbers are assumed.
SLIDE_STIFFNESS_N_PER_M = 700.0
SLIDE_DAMPING = 45.0


def slide_joint_xml(body_name: str) -> str:
    return (
        f'<joint name="susp_{body_name}" type="slide" axis="0 0 1" limited="true" '
        f'range="0 {SLIDE_RANGE_M}" stiffness="{SLIDE_STIFFNESS_N_PER_M}" '
        f'springref="0" damping="{SLIDE_DAMPING}"/>'
    )
