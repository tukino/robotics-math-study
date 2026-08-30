"""One-link planar arm kinematics. / 1リンク平面アームの運動学。"""

from __future__ import annotations

import argparse
import math
from collections.abc import Sequence
from pathlib import Path


def endpoint(angle_radians: float, length: float = 1.0) -> tuple[float, float]:
    """Return the endpoint for a joint angle and link length.

    関節角とリンク長から手先位置を返します。
    """
    if not math.isfinite(angle_radians):
        raise ValueError("angle_radians must be finite / 有限の角度を指定してください")
    if not math.isfinite(length) or length <= 0:
        raise ValueError("length must be positive and finite / リンク長は正の有限値にしてください")

    return length * math.cos(angle_radians), length * math.sin(angle_radians)


def render_svg(angle_radians: float, length: float = 1.0, canvas_size: int = 600) -> str:
    """Return an SVG visualization of a one-link planar arm.

    1リンク平面アームを可視化したSVG文字列を返します。
    """
    x, y = endpoint(angle_radians, length)
    if canvas_size < 200:
        raise ValueError("canvas_size must be at least 200 / 画像サイズは200以上にしてください")

    center = canvas_size / 2
    scale = canvas_size * 0.35 / length
    endpoint_x = center + x * scale
    endpoint_y = center - y * scale
    angle_degrees = math.degrees(angle_radians)

    return f"""<svg xmlns="http://www.w3.org/2000/svg" width="{canvas_size}" height="{canvas_size}" viewBox="0 0 {canvas_size} {canvas_size}">
  <rect width="100%" height="100%" fill="#f8fafc"/>
  <line x1="40" y1="{center}" x2="{canvas_size - 40}" y2="{center}" stroke="#94a3b8" stroke-width="2"/>
  <line x1="{center}" y1="{canvas_size - 40}" x2="{center}" y2="40" stroke="#94a3b8" stroke-width="2"/>
  <text x="{canvas_size - 55}" y="{center - 10}" fill="#475569" font-family="sans-serif">x</text>
  <text x="{center + 10}" y="55" fill="#475569" font-family="sans-serif">y</text>
  <line x1="{center}" y1="{center}" x2="{endpoint_x}" y2="{endpoint_y}" stroke="#2563eb" stroke-width="10" stroke-linecap="round"/>
  <circle cx="{center}" cy="{center}" r="12" fill="#0f172a"/>
  <circle cx="{endpoint_x}" cy="{endpoint_y}" r="12" fill="#f97316"/>
  <text x="24" y="34" fill="#0f172a" font-size="20" font-family="sans-serif">One-link arm / 1リンクアーム</text>
  <text x="24" y="62" fill="#334155" font-size="16" font-family="sans-serif">θ = {angle_degrees:.1f}°, L = {length:g}</text>
  <text x="{endpoint_x + 14}" y="{endpoint_y - 14}" fill="#9a3412" font-size="16" font-family="sans-serif">({x:.3f}, {y:.3f})</text>
</svg>
"""


def save_svg(path: str | Path, angle_radians: float, length: float = 1.0) -> Path:
    """Write an SVG visualization and return its path.

    SVG可視化を書き込み、そのパスを返します。
    """
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(render_svg(angle_radians, length), encoding="utf-8")
    return output_path


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line argument parser.

    コマンドライン引数のパーサーを作成します。
    """
    parser = argparse.ArgumentParser(
        description="Visualize a one-link planar arm / 1リンク平面アームを可視化します"
    )
    parser.add_argument("--angle-deg", type=float, default=45.0, help="joint angle in degrees / 関節角（度）")
    parser.add_argument("--length", type=float, default=1.0, help="link length / リンク長")
    parser.add_argument(
        "--save",
        default="artifacts/one_link_arm.svg",
        help="output SVG path / SVGの保存先",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run the command-line visualization.

    コマンドライン可視化を実行します。
    """
    arguments = build_parser().parse_args(argv)
    angle_radians = math.radians(arguments.angle_deg)
    x, y = endpoint(angle_radians, arguments.length)
    print(f"endpoint = ({x:.6f}, {y:.6f})")

    output_path = save_svg(arguments.save, angle_radians, arguments.length)
    print(f"saved = {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
