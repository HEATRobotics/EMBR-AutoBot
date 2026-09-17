"""Generate the README's calendar timeline using only the Python standard library."""

from datetime import date, datetime
from pathlib import Path
from zoneinfo import ZoneInfo

START = date(2026, 9, 1)
END = date(2027, 8, 31)
OUTPUT = Path(__file__).resolve().parents[2] / "documentation/assets/academic-year-progress.svg"


def render(today: date) -> str:
    total = (END - START).days
    if total <= 0:
        raise ValueError("END must be after START")
    elapsed = min(max((today - START).days, 0), total)
    percent = elapsed / total * 100
    return f'''<svg xmlns="http://www.w3.org/2000/svg" width="720" height="132" viewBox="0 0 720 132" role="img" aria-labelledby="title desc">
  <title id="title">2026–2027 academic-year timeline: {percent:.1f}% elapsed</title>
  <desc id="desc">{elapsed} of {total} days elapsed from {START} to {END}. As of {today}. Calendar time, not engineering completion.</desc>
  <rect width="720" height="132" rx="12" fill="#162330"/>
  <g font-family="Arial, sans-serif" fill="#ffffff">
    <text x="24" y="31" font-size="16" font-weight="bold">2026–2027 ACADEMIC YEAR</text>
    <text x="696" y="31" text-anchor="end" font-size="16">{percent:.1f}% elapsed</text>
    <rect x="24" y="49" width="672" height="18" rx="9" fill="#394b5b"/>
    <rect x="24" y="49" width="{672 * elapsed / total:.2f}" height="18" rx="9" fill="#ff963b"/>
    <text x="24" y="94" font-size="14">Today: {today:%b %d, %Y}</text>
    <text x="696" y="94" text-anchor="end" font-size="14">{END:%b %d, %Y}</text>
    <text x="24" y="117" font-size="12" fill="#c4cdd5">Start: {START:%b %d, %Y} · {elapsed} / {total} days · Calendar time only</text>
  </g>
</svg>
'''


if __name__ == "__main__":
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(render(datetime.now(ZoneInfo("America/Vancouver")).date()), encoding="utf-8")
