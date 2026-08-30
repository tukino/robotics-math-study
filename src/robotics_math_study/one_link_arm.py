"""Interactive one-link planar-arm visualization. / 1リンク平面アームの対話的可視化。"""

from __future__ import annotations

import argparse
import math
from collections.abc import Sequence

import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Circle
from matplotlib.widgets import Button, Slider

from .kinematics import forward_kinematics_1link


def _configure_japanese_font() -> None:
    """Use an installed Japanese font when one is available.

    利用可能な日本語フォントがある場合は描画に使用します。
    """
    installed_fonts = {font.name for font in font_manager.fontManager.ttflist}
    for candidate in ("Meiryo", "Yu Gothic", "BIZ UDPGothic", "Noto Sans JP", "MS Gothic"):
        if candidate in installed_fonts:
            plt.rcParams["font.family"] = [candidate, "DejaVu Sans"]
            return


class OneLinkArmVisualizer:
    """Visualize how a joint angle determines a one-link arm endpoint.

    関節角が1リンクアームの先端位置を決める関係を可視化します。
    """

    def __init__(self, link_length: float = 10.0, initial_angle_deg: float = 30.0) -> None:
        """Create the figure, angle slider, and preset-angle buttons.

        図、角度Slider、代表角度ボタンを作成します。
        """
        # Validate the initial values with the same calculation used by updates.
        # 更新時と同じ座標計算を使って初期値も検証します。
        forward_kinematics_1link(link_length, initial_angle_deg)
        if not 0.0 <= initial_angle_deg <= 360.0:
            raise ValueError("initial_angle_deg must be from 0 to 360 / 初期角度は0〜360度にしてください")

        self.link_length = link_length
        _configure_japanese_font()
        self.figure, self.axes = plt.subplots(figsize=(9, 8))
        self.figure.subplots_adjust(bottom=0.25)
        self._configure_axes()

        x, y = forward_kinematics_1link(self.link_length, initial_angle_deg)
        (self.arm_line,) = self.axes.plot(
            [0.0, x],
            [0.0, y],
            color="tab:blue",
            linewidth=5,
            label="arm vector / アームベクトル",
            zorder=4,
        )
        (self.x_component_line,) = self.axes.plot(
            [0.0, x],
            [0.0, 0.0],
            "--",
            color="tab:orange",
            linewidth=2,
            label="x = L cos(θ)",
            zorder=3,
        )
        (self.y_component_line,) = self.axes.plot(
            [x, x],
            [0.0, y],
            "--",
            color="tab:green",
            linewidth=2,
            label="y = L sin(θ)",
            zorder=3,
        )
        (self.joint_point,) = self.axes.plot(
            0.0,
            0.0,
            "o",
            color="black",
            markersize=10,
            label="joint / 関節",
            zorder=5,
        )
        (self.endpoint_point,) = self.axes.plot(
            x,
            y,
            "o",
            color="tab:red",
            markersize=10,
            label="endpoint / 先端",
            zorder=5,
        )

        self.info_text = self.axes.text(
            0.02,
            0.98,
            "",
            transform=self.axes.transAxes,
            verticalalignment="top",
            fontfamily="monospace",
            fontsize=11,
            bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.9},
        )
        self.endpoint_text = self.axes.annotate(
            "",
            xy=(x, y),
            xytext=(8, 8),
            textcoords="offset points",
            color="tab:red",
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

        # Keep Button objects on self; otherwise they can be garbage-collected.
        # Buttonをselfに保持しないと、ガベージコレクションされる場合があります。
        self.preset_buttons: list[Button] = []
        for index, angle in enumerate((30.0, 45.0, 90.0)):
            button_axes = self.figure.add_axes((0.31 + index * 0.13, 0.035, 0.10, 0.045))
            button = Button(button_axes, f"{angle:.0f}°")
            button.on_clicked(lambda _event, preset=angle: self.set_angle(preset))
            self.preset_buttons.append(button)

        self.axes.legend(loc="lower left", fontsize=9)
        self._update(initial_angle_deg)

    def _configure_axes(self) -> None:
        """Configure axes and draw the fixed radius-L reference circle.

        座標軸を設定し、固定された半径Lの参照円を描画します。
        """
        limit = self.link_length * 1.25
        self.axes.set(
            xlim=(-limit, limit),
            ylim=(-limit, limit),
            aspect="equal",
            xlabel="x",
            ylabel="y",
            title="One-link planar robot arm / 1リンク平面ロボットアーム",
        )
        self.axes.axhline(0.0, color="0.65", linewidth=1)
        self.axes.axvline(0.0, color="0.65", linewidth=1)
        self.axes.grid(True, linestyle=":", alpha=0.45)
        self.axes.add_patch(
            Circle(
                (0.0, 0.0),
                radius=self.link_length,
                fill=False,
                linestyle=":",
                linewidth=2,
                color="0.45",
                label="radius L / 半径Lの円",
            )
        )

    def _on_slider_changed(self, theta_deg: float) -> None:
        """Update the drawing when the Slider value changes.

        Sliderの値が変わったときに描画を更新します。
        """
        self._update(theta_deg)

    def _update(self, theta_deg: float) -> None:
        """Apply one forward-kinematics result to every moving artist.

        1回の順運動学の計算結果を、動くすべての描画要素へ反映します。
        """
        # This is the causal bridge from the angle to the visible arm position.
        # ここが「角度」から「目に見えるアーム位置」への因果関係をつなぎます。
        x, y = forward_kinematics_1link(self.link_length, theta_deg)
        theta_rad = math.radians(theta_deg)
        cos_theta = math.cos(theta_rad)
        sin_theta = math.sin(theta_rad)

        self.arm_line.set_data([0.0, x], [0.0, y])
        self.endpoint_point.set_data([x], [y])
        self.x_component_line.set_data([0.0, x], [0.0, 0.0])
        self.y_component_line.set_data([x, x], [0.0, y])

        self.info_text.set_text(
            f"θ     = {theta_deg:6.1f}°\n"
            f"cos θ = {cos_theta: .4f}\n"
            f"sin θ = {sin_theta: .4f}\n"
            f"x     = L cos θ = {x: .4f}\n"
            f"y     = L sin θ = {y: .4f}"
        )
        self.endpoint_text.xy = (x, y)
        self.endpoint_text.set_text(f"endpoint / 先端\n({x:.3f}, {y:.3f})")
        self.figure.canvas.draw_idle()

    def set_angle(self, theta_deg: float) -> None:
        """Set the angle in degrees and update the Slider and drawing.

        角度を度数法で設定し、Sliderと描画を更新します。
        """
        if not math.isfinite(theta_deg) or not 0.0 <= theta_deg <= 360.0:
            raise ValueError("theta_deg must be from 0 to 360 / 角度は0〜360度にしてください")
        self.angle_slider.set_val(theta_deg)

    def show(self) -> None:
        """Display the interactive Matplotlib window or Notebook output.

        対話的なMatplotlibウィンドウまたはNotebook出力を表示します。
        """
        plt.show()


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line argument parser.

    コマンドライン引数のパーサーを作成します。
    """
    parser = argparse.ArgumentParser(
        description="Visualize one-link forward kinematics / 1リンク順運動学を可視化します"
    )
    parser.add_argument("--length", type=float, default=10.0, help="link length / リンク長")
    parser.add_argument("--angle", type=float, default=30.0, help="initial angle in degrees / 初期角度（度）")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Launch the interactive one-link arm visualization.

    対話的な1リンクアーム可視化を起動します。
    """
    arguments = build_parser().parse_args(argv)
    visualizer = OneLinkArmVisualizer(arguments.length, arguments.angle)
    visualizer.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
