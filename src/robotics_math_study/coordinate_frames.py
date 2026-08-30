"""Interactive local/world coordinate-frame visualization. / ローカル・世界座標系の対話的可視化。"""

from __future__ import annotations

import argparse
import math
from collections.abc import Sequence

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager
from matplotlib.patches import Arc, Circle, FancyArrowPatch
from matplotlib.widgets import Button, Slider

from .coordinate_transforms import rotation_matrix_2d


def _configure_japanese_font() -> None:
    """Use an installed Japanese font when one is available.

    利用可能な日本語フォントがある場合は描画に使用します。
    """
    installed_fonts = {font.name for font in font_manager.fontManager.ttflist}
    for candidate in ("Meiryo", "Yu Gothic", "BIZ UDPGothic", "Noto Sans JP", "MS Gothic"):
        if candidate in installed_fonts:
            plt.rcParams["font.family"] = [candidate, "DejaVu Sans"]
            return


class CoordinateFrameVisualizer:
    """Show a rotating local frame inside a fixed world frame.

    固定された世界座標系の中で回転するローカル座標系を表示します。
    """

    point_p_local = np.array([1.0, 0.0])
    point_q_local = np.array([0.0, 1.0])

    def __init__(self, initial_angle_deg: float = 0.0) -> None:
        """Create the figure, Slider, and preset-angle buttons.

        図、Slider、代表角度ボタンを作成します。
        """
        if not math.isfinite(initial_angle_deg) or not 0.0 <= initial_angle_deg <= 360.0:
            raise ValueError("initial_angle_deg must be from 0 to 360 / 初期角度は0〜360度にしてください")

        _configure_japanese_font()
        self.figure, self.axes = plt.subplots(figsize=(11, 8))
        self.figure.subplots_adjust(bottom=0.25, right=0.70)
        self._configure_axes()

        rotation = rotation_matrix_2d(initial_angle_deg)
        p_world = rotation @ self.point_p_local
        q_world = rotation @ self.point_q_local

        # The arm is fixed along the positive local x' axis, so it rotates with
        # the first local basis vector. A slightly longer line keeps point P visible.
        # アームはローカルx'軸の正方向に固定されているため、第1基底と一緒に
        # 回転します。点Pを見分けやすくするため、アームは少し長く描きます。
        arm_endpoint = 1.25 * p_world
        (self.arm_line,) = self.axes.plot(
            [0.0, arm_endpoint[0]],
            [0.0, arm_endpoint[1]],
            color="lightskyblue",
            linewidth=8,
            label="arm / アーム",
            zorder=2,
        )

        self.local_x_arrow = FancyArrowPatch(
            (0.0, 0.0),
            p_world,
            arrowstyle="-|>",
            mutation_scale=18,
            linewidth=3,
            color="tab:red",
            zorder=4,
        )
        self.local_y_arrow = FancyArrowPatch(
            (0.0, 0.0),
            q_world,
            arrowstyle="-|>",
            mutation_scale=18,
            linewidth=3,
            color="tab:green",
            zorder=4,
        )
        self.axes.add_patch(self.local_x_arrow)
        self.axes.add_patch(self.local_y_arrow)

        (self.point_p_marker,) = self.axes.plot(
            p_world[0],
            p_world[1],
            "o",
            color="tab:red",
            markersize=10,
            label="P: local (1, 0)",
            zorder=5,
        )
        (self.point_q_marker,) = self.axes.plot(
            q_world[0],
            q_world[1],
            "s",
            color="tab:green",
            markersize=9,
            label="Q: local (0, 1)",
            zorder=5,
        )
        (self.p_world_x_projection,) = self.axes.plot(
            [0.0, p_world[0]],
            [0.0, 0.0],
            "--",
            color="tab:red",
            alpha=0.45,
            zorder=1,
        )
        (self.p_world_y_projection,) = self.axes.plot(
            [p_world[0], p_world[0]],
            [0.0, p_world[1]],
            "--",
            color="tab:red",
            alpha=0.45,
            zorder=1,
        )

        self.point_p_text = self.axes.annotate(
            "",
            xy=p_world,
            xytext=(9, 8),
            textcoords="offset points",
            color="tab:red",
            fontsize=10,
        )
        self.point_q_text = self.axes.annotate(
            "",
            xy=q_world,
            xytext=(9, 8),
            textcoords="offset points",
            color="tab:green",
            fontsize=10,
        )
        self.local_x_label = self.axes.text(0.0, 0.0, "x'", color="tab:red", fontsize=13)
        self.local_y_label = self.axes.text(0.0, 0.0, "y'", color="tab:green", fontsize=13)
        self.angle_arc = Arc(
            (0.0, 0.0),
            width=0.55,
            height=0.55,
            theta1=0.0,
            theta2=initial_angle_deg,
            color="tab:purple",
            linewidth=2,
            zorder=3,
        )
        self.axes.add_patch(self.angle_arc)

        self.info_text = self.figure.text(
            0.72,
            0.82,
            "",
            verticalalignment="top",
            fontfamily="monospace",
            fontsize=10,
            bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.92},
            zorder=8,
        )

        slider_axes = self.figure.add_axes((0.18, 0.12, 0.64, 0.04))
        self.angle_slider = Slider(
            ax=slider_axes,
            label="θ (degrees / 度)",
            valmin=0.0,
            valmax=360.0,
            valinit=initial_angle_deg,
            valstep=1.0,
        )
        self.angle_slider.on_changed(self._on_slider_changed)

        self.preset_buttons: list[Button] = []
        for index, angle in enumerate((0.0, 30.0, 45.0, 90.0)):
            button_axes = self.figure.add_axes((0.26 + index * 0.12, 0.035, 0.09, 0.045))
            button = Button(button_axes, f"{angle:.0f}°")
            button.on_clicked(lambda _event, preset=angle: self.set_angle(preset))
            self.preset_buttons.append(button)

        self.axes.legend(loc="lower left", fontsize=9)
        self._update(initial_angle_deg)

    def _configure_axes(self) -> None:
        """Draw the fixed world frame and unit circle.

        固定された世界座標系と単位円を描画します。
        """
        self.axes.set(
            xlim=(-1.55, 1.55),
            ylim=(-1.55, 1.55),
            aspect="equal",
            xlabel="world X / 世界X",
            ylabel="world Y / 世界Y",
            title="Local frame rotating in the world frame / 世界座標系で回転するローカル座標系",
        )
        self.axes.grid(True, linestyle=":", alpha=0.35)
        self.axes.add_patch(
            Circle(
                (0.0, 0.0),
                radius=1.0,
                fill=False,
                linestyle=":",
                linewidth=2,
                color="0.55",
                label="unit circle / 単位円",
            )
        )

        world_x_arrow = FancyArrowPatch(
            (-1.4, 0.0),
            (1.4, 0.0),
            arrowstyle="->",
            mutation_scale=15,
            linewidth=1.5,
            color="0.2",
            zorder=0,
        )
        world_y_arrow = FancyArrowPatch(
            (0.0, -1.4),
            (0.0, 1.4),
            arrowstyle="->",
            mutation_scale=15,
            linewidth=1.5,
            color="0.2",
            zorder=0,
        )
        self.axes.add_patch(world_x_arrow)
        self.axes.add_patch(world_y_arrow)
        self.axes.text(1.42, 0.05, "X", color="0.2", fontsize=13)
        self.axes.text(0.05, 1.42, "Y", color="0.2", fontsize=13)
        self.axes.plot(0.0, 0.0, "o", color="black", markersize=7, zorder=6)

    def _on_slider_changed(self, theta_deg: float) -> None:
        """Update both local basis vectors when the Slider moves.

        Sliderが動いたとき、2つのローカル基底ベクトルを更新します。
        """
        self._update(theta_deg)

    def _update(self, theta_deg: float) -> None:
        """Transform local basis points and apply them to the drawing.

        ローカル基底点を変換し、描画へ反映します。
        """
        rotation = rotation_matrix_2d(theta_deg)

        # These two products are the visual meaning of the matrix columns:
        # R @ (1, 0) is column 1, and R @ (0, 1) is column 2.
        # 次の2つの積が行列の列の視覚的な意味です。
        # R @ (1, 0)は第1列、R @ (0, 1)は第2列になります。
        p_world = rotation @ self.point_p_local
        q_world = rotation @ self.point_q_local
        arm_endpoint = 1.25 * p_world

        self.arm_line.set_data([0.0, arm_endpoint[0]], [0.0, arm_endpoint[1]])
        self.local_x_arrow.set_positions((0.0, 0.0), p_world)
        self.local_y_arrow.set_positions((0.0, 0.0), q_world)
        self.point_p_marker.set_data([p_world[0]], [p_world[1]])
        self.point_q_marker.set_data([q_world[0]], [q_world[1]])
        self.p_world_x_projection.set_data([0.0, p_world[0]], [0.0, 0.0])
        self.p_world_y_projection.set_data(
            [p_world[0], p_world[0]],
            [0.0, p_world[1]],
        )

        self.point_p_text.xy = p_world
        self.point_p_text.set_text(f"P world\n({p_world[0]:.3f}, {p_world[1]:.3f})")
        self.point_q_text.xy = q_world
        self.point_q_text.set_text(f"Q world\n({q_world[0]:.3f}, {q_world[1]:.3f})")
        self.local_x_label.set_position(0.62 * p_world + 0.08 * q_world)
        self.local_y_label.set_position(0.62 * q_world - 0.08 * p_world)
        self.angle_arc.theta2 = theta_deg

        self.info_text.set_text(
            f"θ = {theta_deg:6.1f}°\n"
            f"R = [[{rotation[0, 0]: .3f}, {rotation[0, 1]: .3f}],\n"
            f"     [{rotation[1, 0]: .3f}, {rotation[1, 1]: .3f}]]\n\n"
            f"P local = (1, 0)\n"
            f"P world = ({p_world[0]: .3f}, {p_world[1]: .3f})\n"
            f"        = column 1 of R\n\n"
            f"Q local = (0, 1)\n"
            f"Q world = ({q_world[0]: .3f}, {q_world[1]: .3f})\n"
            f"        = column 2 of R"
        )
        self.figure.canvas.draw_idle()

    def set_angle(self, theta_deg: float) -> None:
        """Set the frame angle in degrees.

        座標系の角度を度数法で設定します。
        """
        if not math.isfinite(theta_deg) or not 0.0 <= theta_deg <= 360.0:
            raise ValueError("theta_deg must be from 0 to 360 / 角度は0〜360度にしてください")
        self.angle_slider.set_val(theta_deg)

    def show(self) -> None:
        """Display the interactive Matplotlib visualization.

        対話的なMatplotlib可視化を表示します。
        """
        plt.show()


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line argument parser.

    コマンドライン引数のパーサーを作成します。
    """
    parser = argparse.ArgumentParser(
        description="Visualize local and world coordinate frames / ローカル・世界座標系を可視化します"
    )
    parser.add_argument("--angle", type=float, default=0.0, help="initial angle in degrees / 初期角度（度）")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Launch the coordinate-frame visualization.

    座標系の可視化を起動します。
    """
    arguments = build_parser().parse_args(argv)
    visualizer = CoordinateFrameVisualizer(arguments.angle)
    visualizer.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
