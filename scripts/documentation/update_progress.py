"""Generate the README goal bar from the booleans in progress.json."""

import json
import re
from hashlib import sha256
from pathlib import Path

CONFIG = Path(__file__).with_name("progress.json")
ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "documentation/assets/goal-progress.svg"
README = ROOT / "README.md"
GOALS = (
    ("teleoperation", "Teleoperation"),
    ("simulation", "Simulation"),
    ("real_world", "Real World"),
    ("autonomy", "Autonomy"),
)


def update_image_link(readme: str, svg: str) -> str:
    """Version the goal bar URL by its content so image caches refresh."""
    version = sha256(svg.encode("utf-8")).hexdigest()[:16]
    updated, count = re.subn(
        r"(!\[EMBR goal progress\]\(documentation/assets/goal-progress\.svg)(?:\?[^)]*)?\)",
        lambda match: f"{match[1]}?v={version})",
        readme,
    )
    if count != 1:
        raise ValueError("Expected exactly one goal progress image link in README.md")
    return updated


def render(progress: dict) -> str:
    """Render independent, equally weighted completion segments."""
    if set(progress) != {key for key, _ in GOALS}:
        raise ValueError("progress.json must contain exactly the configured goal keys")
    if any(type(value) is not bool for value in progress.values()):
        raise ValueError("Each goal must be true or false")
    total = len(GOALS)
    segment_width = (672 - 12 * (total - 1)) / total
    completed = sum(progress.values())
    description = "; ".join(
        f"{label}: {'complete' if progress[key] else 'not complete'}"
        for key, label in GOALS
    )
    segments = []
    for index, (key, label) in enumerate(GOALS):
        x = 24 + index * (segment_width + 12)
        color = "#ff963b" if progress[key] else "#394b5b"
        status = "Complete" if progress[key] else "Not complete"
        segments.append(f'''    <rect x="{x}" y="49" width="{segment_width}" height="18" rx="9" fill="{color}"/>
    <text x="{x + segment_width / 2}" y="94" text-anchor="middle" font-size="16">{label}</text>
    <text x="{x + segment_width / 2}" y="117" text-anchor="middle" font-size="12" fill="#c4cdd5">{status}</text>''')
    bars = "\n".join(segments)
    return f'''<svg xmlns="http://www.w3.org/2000/svg" width="720" height="140" viewBox="0 0 720 140" role="img" aria-labelledby="title desc">
  <title id="title">EMBR goals: {completed} of {total} complete</title>
  <desc id="desc">{description}. Each goal represents {1 / total:.0%} of overall progress.</desc>
  <rect width="720" height="140" rx="12" fill="#162330"/>
  <g font-family="Arial, sans-serif" fill="#ffffff">
    <text x="24" y="31" font-size="16" font-weight="bold">EMBR GOALS</text>
    <text x="696" y="31" text-anchor="end" font-size="16">{completed} / {total} complete ({completed / total:.0%})</text>
{bars}
  </g>
</svg>
'''


if __name__ == "__main__":
    svg = render(json.loads(CONFIG.read_text(encoding="utf-8")))
    readme = update_image_link(README.read_text(encoding="utf-8"), svg)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(svg, encoding="utf-8")
    README.write_text(readme, encoding="utf-8")
