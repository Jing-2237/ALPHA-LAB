"""Run a mission and export CSV telemetry, JSON metrics, and a standalone SVG."""

import argparse
import csv
from dataclasses import asdict
import json
from pathlib import Path

from .model import Config
from .simulation import DEFAULT_ROUTE, simulate


def write_svg(path: Path, rows: list[dict], route):
    points = [(0.0, 0.0)] + [(r["x_m"], r["y_m"]) for r in rows] + list(route)
    lo_x, hi_x = min(x for x, _ in points) - .2, max(x for x, _ in points) + .2
    lo_y, hi_y = min(y for _, y in points) - .2, max(y for _, y in points) + .2
    scale = min(640 / (hi_x - lo_x), 340 / (hi_y - lo_y))
    def xy(x, y):
        return f"{70 + (x - lo_x) * scale:.2f},{430 - (y - lo_y) * scale:.2f}"
    planned = " ".join(xy(x, y) for x, y in [(0, 0), *route])
    actual = " ".join(xy(r["x_m"], r["y_m"]) for r in rows)
    markers = "".join(f'<circle cx="{xy(x,y).split(",")[0]}" cy="{xy(x,y).split(",")[1]}" r="5" fill="#f59e0b"/>' for x, y in route)
    path.write_text(f'''<svg xmlns="http://www.w3.org/2000/svg" width="800" height="500" viewBox="0 0 800 500">
<rect width="800" height="500" rx="16" fill="#101c30"/>
<g font-family="sans-serif" fill="#e2e8f0"><text x="40" y="40" font-size="24">ALPHA-LAB / differential-drive simulation</text>
<text x="40" y="67" font-size="14">Orange: waypoint route · Cyan: closed-loop trajectory · Coordinates in metres</text></g>
<polyline points="{planned}" fill="none" stroke="#f59e0b" stroke-width="2" stroke-dasharray="7 5"/>
<polyline points="{actual}" fill="none" stroke="#22d3ee" stroke-width="3"/>{markers}
<text x="40" y="478" font-family="sans-serif" fill="#94a3b8" font-size="13">Deterministic planar model / ideal brake / no hardware validation</text></svg>''', encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--mission", type=Path)
    parser.add_argument("--output", type=Path, default=Path("outputs/demo"))
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--obstacle-at", type=float)
    args = parser.parse_args()
    try:
        cfg = Config.load(args.config) if args.config else Config()
        route = json.loads(args.mission.read_text(encoding="utf-8"))["waypoints"] if args.mission else DEFAULT_ROUTE
        result = simulate(cfg, route, args.duration, args.obstacle_at)
    except (ValueError, TypeError, KeyError, OSError) as exc:
        parser.error(str(exc))
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / "telemetry.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(result.rows[0]))
        writer.writeheader()
        writer.writerows(result.rows)
    summary = dict(status=result.status, stop_reason=result.reason,
                   completed_waypoints=result.completed_waypoints, total_waypoints=len(route),
                   final_error_m=round(result.final_error, 6), elapsed_s=result.rows[-1]["time_s"],
                   config=asdict(cfg), waypoints=route, obstacle_at_s=args.obstacle_at)
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    write_svg(args.output / "trajectory.svg", result.rows, route)
    print(json.dumps({k: v for k, v in summary.items() if k not in ("config", "waypoints")}, indent=2))
    return 2 if result.status == "timeout" else 0


if __name__ == "__main__":
    raise SystemExit(main())
