"""Interactive two-link arm with multiple coordinate frames. / 複数座標系を持つ2リンクアーム可視化。"""

from __future__ import annotations

import argparse
import math
from collections.abc import Sequence

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager
from matplotlib.patches import FancyArrowPatch
from matplotlib.widgets import Button, Slider

from .coordinate_transforms import frame_axes_world
from .kinematics import TwoLinkPose, forward_kinematics_2link


def _configure_japanese_font() -> None:
    """Use an installed Japanese font when one is available.

    利用可能な日本語フォントがある場合は描画に使用します。
    """
    installed_fonts = {font.name for font in font_manager.fontManager.ttflist}
    for candidate in ("Meiryo", "Yu Gothic", "BIZ UDPGothic", "Noto Sans JP", "MS Gothic"):
        if candidate in installed_fonts:
            plt.rcParams["font.family"] = [candidate, "DejaVu Sans"]
            return


class TwoLinkFrameVisualizer:
    """Visualize World, shoulder, and elbow frames on a two-link arm.

    2リンクアーム上のWorld、肩、肘座標系を可視化します。
    """

    cases = {
        "A": (0.0, 0.0),
        "B": (30.0, 0.0),
        "C": (30.0, 45.0),
        "D": (90.0, -45.0),
    }

    def __init__(
        self,
        link_length_1: float = 5.0,
        link_length_2: float = 4.0,
        initial_theta_1_deg: float = 30.0,
        initial_theta_2_deg: float = 45.0,
    ) -> None:
        """Create links, coordinate frames, Sliders, and case buttons.

        リンク、座標系、Slider、代表ケースボタンを作成します。
        """
        pose = forward_kinematics_2link(
            link_length_1,
            link_length_2,
            initial_theta_1_deg,
            initial_theta_2_deg,
        )
        self._validate_slider_angle(initial_theta_1_deg, "initial_theta_1_deg")
        self._validate_slider_angle(initial_theta_2_deg, "initial_theta_2_deg")

        self.link_length_1 = link_length_1
        self.link_length_2 = link_length_2
        self.frame_axis_length = 0.20 * (link_length_1 + link_length_2)

        _configure_japanese_font()
        self.figure, self.axes = plt.subplots(figsize=(12, 8))
        self.figure.subplots_adjust(bottom=0.30, right=0.68)
        self._configure_axes()

        shoulder = np.asarray(pose["shoulder_world"])
        elbow = np.asarray(pose["elbow_world"])
        wrist = np.asarray(pose["wrist_world"])

        (self.upper_arm_line,) = self.axes.plot(
            [shoulder[0], elbow[0]],
            [shoulder[1], elbow[1]],
            color="tab:blue",
            linewidth=7,
            label="upper arm L1 / 上腕",
            zorder=3,
        )
        (self.forearm_line,) = self.axes.plot(
            [elbow[0], wrist[0]],
            [elbow[1], wrist[1]],
            color="tab:orange",
            linewidth=7,
            label="forearm L2 / 前腕",
            zorder=3,
        )
        (self.shoulder_marker,) = self.axes.plot(
            shoulder[0],
            shoulder[1],
            "o",
            color="black",
            markersize=10,
            label="shoulder / 肩",
            zorder=6,
        )
        (self.elbow_marker,) = self.axes.plot(
            elbow[0],
            elbow[1],
            "o",
            color="tab:purple",
            markersize=11,
            label="elbow / 肘",
            zorder=6,
        )
        (self.wrist_marker,) = self.axes.plot(
            wrist[0],
            wrist[1],
            "o",
            color="tab:red",
            markersize=10,
            label="wrist / 手首",
            zorder=6,
        )

        self.shoulder_x_arrow = self._add_frame_arrow("tab:red")
        self.shoulder_y_arrow = self._add_frame_arrow("tab:green")
        self.elbow_x_arrow = self._add_frame_arrow("crimson")
        self.elbow_y_arrow = self._add_frame_arrow("seagreen")

        self.shoulder_x_label = self.axes.text(0.0, 0.0, "x_s", color="tab:red", fontsize=11)
        self.shoulder_y_label = self.axes.text(0.0, 0.0, "y_s", color="tab:green", fontsize=11)
        self.elbow_x_label = self.axes.text(0.0, 0.0, "x_e", color="crimson", fontsize=11)
        self.elbow_y_label = self.axes.text(0.0, 0.0, "y_e", color="seagreen", fontsize=11)

        self.shoulder_text = self.axes.annotate(
            "Shoulder Origin\nShoulder local: (0,0)",
            xy=shoulder,
            xytext=(8, -36),
            textcoords="offset points",
            color="black",
            fontsize=9,
        )
        self.elbow_text = self.axes.annotate(
            "",
            xy=elbow,
            xytext=(22, -58),
            textcoords="offset points",
            color="tab:purple",
            fontsize=10,
            bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.88},
            arrowprops={"arrowstyle": "->", "color": "tab:purple", "linewidth": 1.0},
            zorder=8,
        )
        self.wrist_text = self.axes.annotate(
            "",
            xy=wrist,
            xytext=(10, 8),
            textcoords="offset points",
            color="tab:red",
            fontsize=10,
        )
        self.info_text = self.figure.text(
            0.71,
            0.84,
            "",
            verticalalignment="top",
            fontsize=10,
            bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.94},
        )

        theta_1_axes = self.figure.add_axes((0.18, 0.18, 0.58, 0.035))
        theta_2_axes = self.figure.add_axes((0.18, 0.12, 0.58, 0.035))
        self.theta_1_slider = Slider(
            theta_1_axes,
            "theta1 / 肩角度",
            -180.0,
            180.0,
            valinit=initial_theta_1_deg,
            valstep=1.0,
        )
        self.theta_2_slider = Slider(
            theta_2_axes,
            "theta2 / 肘相対角度",
            -180.0,
            180.0,
            valinit=initial_theta_2_deg,
            valstep=1.0,
        )
        self.theta_1_slider.on_changed(self._on_slider_changed)
        self.theta_2_slider.on_changed(self._on_slider_changed)

        self.case_buttons: list[Button] = []
        for index, (case_name, angles) in enumerate(self.cases.items()):
            button_axes = self.figure.add_axes((0.26 + index * 0.13, 0.035, 0.11, 0.05))
            button = Button(
                button_axes,
                f"Case {case_name}\n({angles[0]:.0f}°, {angles[1]:.0f}°)",
            )
            button.on_clicked(lambda _event, name=case_name: self.set_case(name))
            self.case_buttons.append(button)

        self.axes.legend(loc="lower left", fontsize=9)
        self._update(initial_theta_1_deg, initial_theta_2_deg)

    @staticmethod
    def _validate_slider_angle(theta_deg: float, name: str) -> None:
        """Validate an angle against the Slider's supported range.

        角度がSliderの対応範囲内か検証します。
        """
        if not math.isfinite(theta_deg) or not -180.0 <= theta_deg <= 180.0:
            raise ValueError(f"{name} must be from -180 to 180 / 角度は-180〜180度にしてください")

    def _configure_axes(self) -> None:
        """Draw the fixed World frame.

        固定されたWorld座標系を描画します。
        """
        limit = 1.20 * (self.link_length_1 + self.link_length_2)
        self.axes.set(
            xlim=(-limit, limit),
            ylim=(-limit, limit),
            aspect="equal",
            xlabel="World X",
            ylabel="World Y",
            title="Two-link arm with multiple Origins / 複数Originを持つ2リンクアーム",
        )
        self.axes.grid(True, linestyle=":", alpha=0.35)
        self.axes.axhline(0.0, color="0.25", linewidth=1.5, zorder=0)
        self.axes.axvline(0.0, color="0.25", linewidth=1.5, zorder=0)
        self.axes.text(limit * 0.96, limit * 0.03, "X", color="0.2", fontsize=12)
        self.axes.text(limit * 0.03, limit * 0.96, "Y", color="0.2", fontsize=12)

    def _add_frame_arrow(self, color: str) -> FancyArrowPatch:
        """Add one reusable local-axis arrow.

        再利用可能なローカル座標軸の矢印を1本追加します。
        """
        arrow = FancyArrowPatch(
            (0.0, 0.0),
            (0.0, 0.0),
            arrowstyle="-|>",
            mutation_scale=15,
            linewidth=2.2,
            color=color,
            zorder=5,
        )
        self.axes.add_patch(arrow)
        return arrow

    def _on_slider_changed(self, _value: float) -> None:
        """Update the complete arm and all frames after either Slider changes.

        どちらかのSlider変更後に、アームと全座標系を更新します。
        """
        self._update(self.theta_1_slider.val, self.theta_2_slider.val)

    def _update(self, theta_1_deg: float, theta_2_deg: float) -> None:
        """Apply one two-link forward-kinematics result to the drawing.

        1回の2リンク順運動学の結果を描画へ反映します。
        """
        pose = forward_kinematics_2link(
            self.link_length_1,
            self.link_length_2,
            theta_1_deg,
            theta_2_deg,
        )
        shoulder = np.asarray(pose["shoulder_world"])
        elbow = np.asarray(pose["elbow_world"])
        wrist = np.asarray(pose["wrist_world"])

        # The shoulder frame follows link 1. The elbow frame is located at the
        # elbow and follows link 2, whose World angle is theta1 + theta2.
        # 肩フレームは第1リンクに従います。肘フレームのOriginは肘にあり、
        # World角度がtheta1 + theta2となる第2リンクに従います。
        shoulder_x, shoulder_y = frame_axes_world(theta_1_deg)
        elbow_x, elbow_y = frame_axes_world(theta_1_deg + theta_2_deg)

        self.upper_arm_line.set_data([shoulder[0], elbow[0]], [shoulder[1], elbow[1]])
        self.forearm_line.set_data([elbow[0], wrist[0]], [elbow[1], wrist[1]])
        self.elbow_marker.set_data([elbow[0]], [elbow[1]])
        self.wrist_marker.set_data([wrist[0]], [wrist[1]])

        self._set_frame(
            shoulder,
            shoulder_x,
            shoulder_y,
            self.shoulder_x_arrow,
            self.shoulder_y_arrow,
            self.shoulder_x_label,
            self.shoulder_y_label,
        )
        self._set_frame(
            elbow,
            elbow_x,
            elbow_y,
            self.elbow_x_arrow,
            self.elbow_y_arrow,
            self.elbow_x_label,
            self.elbow_y_label,
        )

        self.elbow_text.xy = elbow
        self.elbow_text.set_text(
            "Elbow local: (0,0)\n"
            f"Elbow in World: ({elbow[0]:.3f}, {elbow[1]:.3f})"
        )
        self.wrist_text.xy = wrist
        self.wrist_text.set_text(f"Wrist in World\n({wrist[0]:.3f}, {wrist[1]:.3f})")
        self.info_text.set_text(self._format_info(pose, theta_1_deg, theta_2_deg))
        self.figure.canvas.draw_idle()

    def _set_frame(
        self,
        origin: np.ndarray,
        x_direction: np.ndarray,
        y_direction: np.ndarray,
        x_arrow: FancyArrowPatch,
        y_arrow: FancyArrowPatch,
        x_label: plt.Text,
        y_label: plt.Text,
    ) -> None:
        """Place a pair of local axes at one joint Origin.

        1つの関節Originへローカル座標軸の組を配置します。
        """
        x_endpoint = origin + self.frame_axis_length * x_direction
        y_endpoint = origin + self.frame_axis_length * y_direction
        x_arrow.set_positions(origin, x_endpoint)
        y_arrow.set_positions(origin, y_endpoint)
        x_label.set_position(origin + 1.08 * self.frame_axis_length * x_direction)
        y_label.set_position(origin + 1.08 * self.frame_axis_length * y_direction)

    def _format_info(
        self,
        pose: TwoLinkPose,
        theta_1_deg: float,
        theta_2_deg: float,
    ) -> str:
        """Format the angle, Origin, and vector-addition learning panel.

        角度、Origin、ベクトル加算の学習パネルを整形します。
        """
        elbow = pose["elbow_world"]
        wrist = pose["wrist_world"]
        forearm_vector = (wrist[0] - elbow[0], wrist[1] - elbow[1])
        return (
            f"theta1 = {theta_1_deg:7.1f}°\n"
            f"theta2 = {theta_2_deg:7.1f}° (relative / 相対)\n"
            f"forearm World angle\n"
            f"       = theta1 + theta2\n"
            f"       = {pose['forearm_angle_world_deg']:7.1f}°\n\n"
            f"Shoulder World = (0.000, 0.000)\n"
            f"Elbow local    = (0.000, 0.000)\n"
            f"Elbow World    = ({elbow[0]: .3f}, {elbow[1]: .3f})\n"
            f"Wrist World    = ({wrist[0]: .3f}, {wrist[1]: .3f})\n\n"
            f"Shoulder -> Elbow\n"
            f"  = ({elbow[0]: .3f}, {elbow[1]: .3f})\n"
            f"Elbow -> Wrist\n"
            f"  = ({forearm_vector[0]: .3f}, {forearm_vector[1]: .3f})\n"
            f"Their World-vector sum = Wrist"
        )

    def set_angles(self, theta_1_deg: float, theta_2_deg: float) -> None:
        """Set both joint angles without performing an intermediate update.

        中間状態を描画せず、両方の関節角度を設定します。
        """
        self._validate_slider_angle(theta_1_deg, "theta_1_deg")
        self._validate_slider_angle(theta_2_deg, "theta_2_deg")

        self.theta_1_slider.eventson = False
        self.theta_2_slider.eventson = False
        try:
            self.theta_1_slider.set_val(theta_1_deg)
            self.theta_2_slider.set_val(theta_2_deg)
        finally:
            self.theta_1_slider.eventson = True
            self.theta_2_slider.eventson = True
        self._update(theta_1_deg, theta_2_deg)

    def set_case(self, case_name: str) -> None:
        """Select one of the learning cases A through D.

        学習ケースA〜Dのいずれかを選択します。
        """
        normalized_name = case_name.upper()
        if normalized_name not in self.cases:
            raise ValueError("case_name must be A, B, C, or D / ケース名はA、B、C、Dです")
        self.set_angles(*self.cases[normalized_name])

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
        description="Visualize multiple Origins on a two-link arm / 2リンクアームの複数Originを可視化します"
    )
    parser.add_argument("--length-1", type=float, default=5.0, help="upper-arm length L1 / 上腕長L1")
    parser.add_argument("--length-2", type=float, default=4.0, help="forearm length L2 / 前腕長L2")
    parser.add_argument("--theta-1", type=float, default=30.0, help="shoulder angle / 肩角度")
    parser.add_argument("--theta-2", type=float, default=45.0, help="relative elbow angle / 肘相対角度")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Launch the two-link coordinate-frame visualization.

    2リンク座標系の可視化を起動します。
    """
    arguments = build_parser().parse_args(argv)
    visualizer = TwoLinkFrameVisualizer(
        arguments.length_1,
        arguments.length_2,
        arguments.theta_1,
        arguments.theta_2,
    )
    visualizer.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
