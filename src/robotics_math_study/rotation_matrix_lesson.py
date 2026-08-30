"""Interactive lessons for rotation matrices as frame transforms. / 座標変換としての回転行列教材。"""

from __future__ import annotations

import math

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager
from matplotlib.patches import FancyArrowPatch
from matplotlib.widgets import Slider

from .coordinate_transforms import frame_axes_world, rotation_matrix_2d
from .vector_kinematics import forward_kinematics_2link_vectors, link_vector_world


def _configure_japanese_font() -> None:
    """Use an installed Japanese font when available. / 利用可能な日本語フォントを使用します。"""
    installed_fonts = {font.name for font in font_manager.fontManager.ttflist}
    for candidate in ("Meiryo", "Yu Gothic", "BIZ UDPGothic", "Noto Sans JP", "MS Gothic"):
        if candidate in installed_fonts:
            plt.rcParams["font.family"] = [candidate, "DejaVu Sans"]
            return


def _mark_interactive_figure(figure: plt.Figure) -> None:
    """Give interactive figures a distinct color and environment label.

    対話図に固有の背景色と、実行環境を示すラベルを付けます。
    """
    figure.patch.set_facecolor("#fff8dc")
    figure.text(
        0.5,
        0.995,
        "INTERACTIVE IN JUPYTER / JupyterではSlider操作可能  |  "
        "STATIC PREVIEW IN NOTEBOOKLM / NotebookLMでは静止画像",
        ha="center",
        va="top",
        weight="bold",
        color="#5f4300",
        bbox={
            "boxstyle": "round,pad=0.35",
            "facecolor": "#ffd166",
            "edgecolor": "#b77900",
            "linewidth": 1.5,
        },
    )


def _mark_slider_axes(slider_axes: plt.Axes) -> None:
    """Color a Slider area as an interactive control. / Slider領域を操作部として着色します。"""
    slider_axes.set_facecolor("#ffe8a3")


def _arrow(axes: plt.Axes, color: str, width: float = 2.2) -> FancyArrowPatch:
    """Create a reusable arrow artist. / 再利用可能な矢印Artistを作ります。"""
    artist = FancyArrowPatch(
        (0.0, 0.0), (0.0, 0.0), arrowstyle="-|>", mutation_scale=15,
        linewidth=width, color=color, zorder=4,
    )
    axes.add_patch(artist)
    return artist


def _set_arrow(artist: FancyArrowPatch, start: np.ndarray, vector: np.ndarray) -> None:
    """Place an arrow using a start point and vector. / 始点とベクトルで矢印を配置します。"""
    artist.set_positions(start, start + vector)


class RulerViewComparisonVisualizer:
    """Compare one physical arrow in ruler-fixed and desk-fixed coordinates.

    同じ物理的な矢印を、定規固定座標と机固定座標で並べて比較します。
    """

    def __init__(self, ruler_length: float = 5.0, initial_theta_deg: float = 30.0) -> None:
        """Build the side-by-side comparison and angle Slider.

        左右比較図と角度Sliderを作成します。
        """
        link_vector_world(ruler_length, initial_theta_deg)
        if not -180.0 <= initial_theta_deg <= 180.0:
            raise ValueError("initial_theta_deg must be from -180 to 180 / 角度は-180〜180度です")
        self.ruler_length = ruler_length
        _configure_japanese_font()
        self.figure, (self.ruler_axes, self.desk_axes) = plt.subplots(1, 2, figsize=(13, 7))
        _mark_interactive_figure(self.figure)
        self.figure.subplots_adjust(bottom=0.18, top=0.66, wspace=0.28)
        limit = 1.25 * ruler_length

        self.ruler_axes.set(
            xlim=(-1.0, limit), ylim=(-limit / 2, limit / 2), aspect="equal",
            xlabel="forward / 定規の前", ylabel="side / 定規の横",
            title="View A: coordinates printed on the ruler\n見方A：定規に印刷された座標",
        )
        self.ruler_axes.axhline(0, color="tab:red", linewidth=1.5)
        self.ruler_axes.axvline(0, color="tab:green", linewidth=1.5)
        self.ruler_axes.grid(True, linestyle=":", alpha=0.3)
        local_arrow = _arrow(self.ruler_axes, "tab:blue", 4.0)
        _set_arrow(local_arrow, np.zeros(2), np.array([ruler_length, 0.0]))
        self.ruler_axes.plot(ruler_length, 0, "o", color="tab:blue", markersize=9)
        self.ruler_axes.text(
            ruler_length * 0.48, 0.35,
            "physical arrow A / 実物の矢印A\n[forward, side] = [L, 0]",
            ha="center",
        )
        self.local_invariant_text = self.ruler_axes.text(
            0.03, 0.05, "", transform=self.ruler_axes.transAxes,
            bbox={"boxstyle": "round", "facecolor": "aliceblue", "alpha": 0.95},
        )

        self.desk_axes.set(
            xlim=(-limit, limit), ylim=(-limit, limit), aspect="equal",
            xlabel="horizontal X / 机の横", ylabel="vertical Y / 机の縦",
            title="View B: coordinates printed on the desk\n見方B：机に印刷された座標",
        )
        self.desk_axes.axhline(0, color="0.3", linewidth=1.5)
        self.desk_axes.axvline(0, color="0.3", linewidth=1.5)
        self.desk_axes.grid(True, linestyle=":", alpha=0.3)
        self.world_arrow = _arrow(self.desk_axes, "tab:blue", 4.0)
        (self.world_endpoint,) = self.desk_axes.plot([], [], "o", color="tab:blue", markersize=9)
        (self.x_component,) = self.desk_axes.plot([], [], "--", color="tab:red", linewidth=2)
        (self.y_component,) = self.desk_axes.plot([], [], "--", color="tab:green", linewidth=2)
        self.world_label = self.desk_axes.text(0, 0, "")
        self.translation_text = self.figure.text(
            0.50, 0.90, "", ha="center", va="top",
            bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.96},
        )

        slider_axes = self.figure.add_axes((0.24, 0.065, 0.52, 0.04))
        _mark_slider_axes(slider_axes)
        self.theta_slider = Slider(
            slider_axes, "turn ruler / 定規を回す theta", -180, 180,
            valinit=initial_theta_deg, valstep=1,
        )
        self.theta_slider.on_changed(self._update)
        self._update(initial_theta_deg)

    def _update(self, theta_deg: float) -> None:
        """Update only the desk description of the unchanged physical arrow.

        変わらない実物の矢印について、机基準の表現だけを更新します。
        """
        local = np.array([self.ruler_length, 0.0])
        rotation = rotation_matrix_2d(theta_deg)
        world = rotation @ local
        origin = np.zeros(2)
        _set_arrow(self.world_arrow, origin, world)
        self.world_endpoint.set_data([world[0]], [world[1]])
        self.x_component.set_data([0.0, world[0]], [0.0, 0.0])
        self.y_component.set_data([world[0], world[0]], [0.0, world[1]])
        self.world_label.set_position(world + np.array([0.18, 0.18]))
        self.world_label.set_text(
            "same physical arrow A / 同じ実物の矢印A\n"
            f"[horizontal, vertical] = [{world[0]:.2f}, {world[1]:.2f}]"
        )
        self.local_invariant_text.set_text(
            f"Ruler turned to theta = {theta_deg:.0f} deg\n"
            "but its own description stays [L, 0]\n"
            f"定規を{theta_deg:.0f}度に回しても、定規自身の表現は[L, 0]のまま"
        )
        first_column = rotation[:, 0]
        second_column = rotation[:, 1]
        self.translation_text.set_text(
            "R records where ruler-forward and ruler-side point on the desk\n"
            "Rは『定規の前・横』が机上で向く方向を記録する\n"
            f"R @ [L, 0] = L x column 1 + 0 x column 2 = "
            f"[{world[0]:.2f}, {world[1]:.2f}]\n"
            f"column 1={np.array2string(first_column, precision=2)}, "
            f"column 2={np.array2string(second_column, precision=2)}"
        )
        self.figure.canvas.draw_idle()

    def set_angle(self, theta_deg: float) -> None:
        """Set the ruler angle from code. / コードから定規の角度を設定します。"""
        if not math.isfinite(theta_deg) or not -180.0 <= theta_deg <= 180.0:
            raise ValueError("theta_deg must be from -180 to 180 / 角度は-180〜180度です")
        self.theta_slider.set_val(theta_deg)


class OneLinkRotationMatrixVisualizer:
    """Show ``R(theta) @ [L, 0]`` as a one-link World vector.

    ``R(theta) @ [L, 0]``を1リンクのWorldベクトルとして表示します。
    """

    def __init__(self, link_length: float = 5.0, initial_theta_deg: float = 30.0) -> None:
        """Build the one-link figure and angle Slider. / 1リンク図と角度Sliderを作ります。"""
        link_vector_world(link_length, initial_theta_deg)
        if not -180.0 <= initial_theta_deg <= 180.0:
            raise ValueError("initial_theta_deg must be from -180 to 180 / 角度は-180〜180度です")
        self.link_length = link_length
        _configure_japanese_font()
        self.figure, self.axes = plt.subplots(figsize=(11, 7))
        _mark_interactive_figure(self.figure)
        self.figure.subplots_adjust(bottom=0.18, top=0.88, right=0.67)
        limit = 1.35 * link_length
        self.axes.set(xlim=(-limit, limit), ylim=(-limit, limit), aspect="equal",
                      xlabel="desk/World horizontal X / 机の横方向X",
                      ylabel="desk/World vertical Y / 机の縦方向Y",
                      title="The same ruler seen from the ruler and from the desk\n"
                            "同じ定規を『定規基準』と『机基準』で見る")
        self.axes.axhline(0, color="0.3", linewidth=1.3)
        self.axes.axvline(0, color="0.3", linewidth=1.3)
        self.axes.grid(True, linestyle=":", alpha=0.35)
        self.local_x_arrow = _arrow(self.axes, "tab:red")
        self.local_y_arrow = _arrow(self.axes, "tab:green")
        self.world_vector_arrow = _arrow(self.axes, "tab:blue", 4.0)
        (self.endpoint,) = self.axes.plot([], [], "o", color="tab:blue", markersize=9)
        self.local_x_label = self.axes.text(0, 0, "forward x' / 前", color="tab:red")
        self.local_y_label = self.axes.text(0, 0, "side y' / 横", color="tab:green")
        self.link_label = self.axes.text(0, 0, "")
        self.endpoint_label = self.axes.annotate(
            "endpoint / 先端", (0, 0), xytext=(7, 7), textcoords="offset points"
        )
        self.info = self.figure.text(0.70, 0.83, "", va="top",
                                     bbox={"boxstyle": "round", "facecolor": "white"})
        slider_axes = self.figure.add_axes((0.20, 0.07, 0.52, 0.04))
        _mark_slider_axes(slider_axes)
        self.theta_slider = Slider(slider_axes, "theta / 角度", -180, 180,
                                   valinit=initial_theta_deg, valstep=1)
        self.theta_slider.on_changed(self._on_change)
        self._update(initial_theta_deg)

    def _on_change(self, theta_deg: float) -> None:
        """Update after Slider movement. / Slider操作後に図を更新します。"""
        self._update(theta_deg)

    def _update(self, theta_deg: float) -> None:
        """Apply one rotation result to all artists. / 1回の回転結果を全Artistへ反映します。"""
        rotation = rotation_matrix_2d(theta_deg)
        v_local = np.array([self.link_length, 0.0])
        v_world = rotation @ v_local
        x_axis, y_axis = frame_axes_world(theta_deg)
        axis_length = 0.38 * self.link_length
        origin = np.zeros(2)
        _set_arrow(self.local_x_arrow, origin, axis_length * x_axis)
        _set_arrow(self.local_y_arrow, origin, axis_length * y_axis)
        _set_arrow(self.world_vector_arrow, origin, v_world)
        self.endpoint.set_data([v_world[0]], [v_world[1]])
        self.endpoint_label.xy = v_world
        self.local_x_label.set_position(1.08 * axis_length * x_axis)
        self.local_y_label.set_position(1.08 * axis_length * y_axis)
        self.link_label.set_position(0.55 * v_world)
        self.link_label.set_text("ruler/arm / 定規・腕\nforward L, side 0 = [L, 0]")
        self.info.set_text(
            f"theta = {theta_deg:6.1f} deg\n\n"
            f"R(theta) =\n{np.array2string(rotation, precision=3)}\n\n"
            f"ruler view / 定規基準\n"
            f"  [forward, side] = [{v_local[0]:.3f}, {v_local[1]:.3f}]\n\n"
            f"desk view / 机基準\n"
            f"  [horizontal, vertical] = [{v_world[0]:.3f}, {v_world[1]:.3f}]\n\n"
            "v_world = R(theta) @ v_local"
        )
        self.figure.canvas.draw_idle()

    def set_angle(self, theta_deg: float) -> None:
        """Set a learning angle from code. / コードから確認角度を設定します。"""
        if not math.isfinite(theta_deg) or not -180.0 <= theta_deg <= 180.0:
            raise ValueError("theta_deg must be from -180 to 180 / 角度は-180〜180度です")
        self.theta_slider.set_val(theta_deg)


class TwoLinkRotationMatrixVisualizer:
    """Show two rotated World vectors added head-to-tail.

    回転した2本のWorldベクトルを先端同士でつないで表示します。
    """

    def __init__(self, link_length_1: float = 5.0, link_length_2: float = 4.0,
                 initial_theta_1_deg: float = 30.0, initial_theta_2_deg: float = 45.0) -> None:
        """Build the two-link vector-addition figure. / 2リンクのベクトル加算図を作ります。"""
        forward_kinematics_2link_vectors(link_length_1, link_length_2,
                                         initial_theta_1_deg, initial_theta_2_deg)
        if not all(-180.0 <= value <= 180.0
                   for value in (initial_theta_1_deg, initial_theta_2_deg)):
            raise ValueError("initial angles must be from -180 to 180 / 初期角度は-180〜180度です")
        self.link_length_1 = link_length_1
        self.link_length_2 = link_length_2
        _configure_japanese_font()
        self.figure, self.axes = plt.subplots(figsize=(12, 8))
        _mark_interactive_figure(self.figure)
        self.figure.subplots_adjust(bottom=0.23, top=0.88, right=0.66)
        limit = 1.25 * (link_length_1 + link_length_2)
        self.axes.set(xlim=(-limit, limit), ylim=(-limit, limit), aspect="equal",
                      xlabel="desk/World horizontal X / 机の横方向X",
                      ylabel="desk/World vertical Y / 机の縦方向Y",
                      title="Walk along two rulers to reach the wrist\n"
                            "2本の定規に沿って進み、手首へ到達する")
        self.axes.axhline(0, color="0.3", linewidth=1.3)
        self.axes.axvline(0, color="0.3", linewidth=1.3)
        self.axes.grid(True, linestyle=":", alpha=0.35)
        self.v1_arrow = _arrow(self.axes, "tab:blue", 4.0)
        self.v2_arrow = _arrow(self.axes, "tab:orange", 4.0)
        self.shoulder_x = _arrow(self.axes, "crimson")
        self.shoulder_y = _arrow(self.axes, "seagreen")
        self.elbow_x = _arrow(self.axes, "crimson")
        self.elbow_y = _arrow(self.axes, "seagreen")
        (self.joints,) = self.axes.plot([], [], "o-", color="0.2", alpha=0.3, markersize=8)
        self.vector_labels = [self.axes.text(0, 0, ""), self.axes.text(0, 0, "")]
        self.joint_labels = [
            self.axes.annotate("Shoulder / 肩", (0, 0), xytext=(6, -18), textcoords="offset points"),
            self.axes.annotate("Elbow / 肘", (0, 0), xytext=(-78, 18), textcoords="offset points"),
            self.axes.annotate("Wrist / 手首", (0, 0), xytext=(7, 7), textcoords="offset points"),
        ]
        self.frame_labels = [self.axes.text(0, 0, "x_s", color="crimson"),
                             self.axes.text(0, 0, "y_s", color="seagreen"),
                             self.axes.text(0, 0, "x_e", color="crimson"),
                             self.axes.text(0, 0, "y_e", color="seagreen")]
        self.info = self.figure.text(0.69, 0.86, "", va="top",
                                     bbox={"boxstyle": "round", "facecolor": "white"})
        ax1 = self.figure.add_axes((0.18, 0.11, 0.52, 0.035))
        ax2 = self.figure.add_axes((0.18, 0.055, 0.52, 0.035))
        _mark_slider_axes(ax1)
        _mark_slider_axes(ax2)
        self.theta_1_slider = Slider(ax1, "theta1 / 肩", -180, 180,
                                     valinit=initial_theta_1_deg, valstep=1)
        self.theta_2_slider = Slider(ax2, "theta2 / 肘相対", -180, 180,
                                     valinit=initial_theta_2_deg, valstep=1)
        self.theta_1_slider.on_changed(self._on_change)
        self.theta_2_slider.on_changed(self._on_change)
        self._update(initial_theta_1_deg, initial_theta_2_deg)

    def _on_change(self, _value: float) -> None:
        """Update after either Slider moves. / どちらかのSlider操作後に更新します。"""
        self._update(self.theta_1_slider.val, self.theta_2_slider.val)

    def _update(self, theta_1_deg: float, theta_2_deg: float) -> None:
        """Apply vector forward kinematics to the drawing. / ベクトル順運動学を図へ反映します。"""
        pose = forward_kinematics_2link_vectors(
            self.link_length_1, self.link_length_2, theta_1_deg, theta_2_deg)
        shoulder = pose["p_shoulder_world"]
        elbow = pose["p_elbow_world"]
        wrist = pose["p_wrist_world"]
        _set_arrow(self.v1_arrow, shoulder, pose["v1_world"])
        _set_arrow(self.v2_arrow, elbow, pose["v2_world"])
        self.joints.set_data([shoulder[0], elbow[0], wrist[0]],
                             [shoulder[1], elbow[1], wrist[1]])
        axis_length = 0.14 * (self.link_length_1 + self.link_length_2)
        sx, sy = frame_axes_world(theta_1_deg)
        ex, ey = frame_axes_world(theta_1_deg + theta_2_deg)
        for artist, origin, direction in (
            (self.shoulder_x, shoulder, sx), (self.shoulder_y, shoulder, sy),
            (self.elbow_x, elbow, ex), (self.elbow_y, elbow, ey),
        ):
            _set_arrow(artist, origin, axis_length * direction)
        for label, origin, direction in zip(
            self.frame_labels, (shoulder, shoulder, elbow, elbow), (sx, sy, ex, ey), strict=True):
            label.set_position(origin + 1.08 * axis_length * direction)
        self.joint_labels[0].xy = shoulder
        self.joint_labels[1].xy = elbow
        self.joint_labels[2].xy = wrist
        v1_perpendicular = np.array([-pose["v1_world"][1], pose["v1_world"][0]])
        v2_perpendicular = np.array([-pose["v2_world"][1], pose["v2_world"][0]])
        v1_perpendicular /= np.linalg.norm(v1_perpendicular)
        v2_perpendicular /= np.linalg.norm(v2_perpendicular)
        self.vector_labels[0].set_position(
            shoulder + 0.50 * pose["v1_world"] - 0.55 * v1_perpendicular
        )
        self.vector_labels[0].set_text("walk 1 / 移動1")
        self.vector_labels[1].set_position(
            elbow + 0.50 * pose["v2_world"] + 0.55 * v2_perpendicular
        )
        self.vector_labels[1].set_text("walk 2 / 移動2")
        self.info.set_text(
            f"theta1 = {theta_1_deg:6.1f} deg\n"
            f"theta1+theta2 = {theta_1_deg + theta_2_deg:6.1f} deg\n\n"
            "Each ruler says: forward by L, side 0\n"
            "各定規: 前へL、横へ0\n"
            f"v1_local = {np.array2string(pose['v1_local'], precision=3)}\n"
            f"v1_world = {np.array2string(pose['v1_world'], precision=3)}\n"
            f"v2_local = {np.array2string(pose['v2_local'], precision=3)}\n"
            f"v2_world = {np.array2string(pose['v2_world'], precision=3)}\n\n"
            f"p_elbow = {np.array2string(elbow, precision=3)}\n"
            f"p_wrist = {np.array2string(wrist, precision=3)}\n\n"
            "desk-view walk 1 + walk 2 = wrist\n"
            "机基準の移動1 + 移動2 = 手首\n"
            "p_wrist = v1_world + v2_world"
        )
        self.figure.canvas.draw_idle()

    def set_angles(self, theta_1_deg: float, theta_2_deg: float) -> None:
        """Set both learning angles. / 2つの確認角度を設定します。"""
        if not all(math.isfinite(value) and -180 <= value <= 180
                   for value in (theta_1_deg, theta_2_deg)):
            raise ValueError("angles must be from -180 to 180 / 角度は-180〜180度です")
        self.theta_1_slider.eventson = False
        self.theta_2_slider.eventson = False
        try:
            self.theta_1_slider.set_val(theta_1_deg)
            self.theta_2_slider.set_val(theta_2_deg)
        finally:
            self.theta_1_slider.eventson = True
            self.theta_2_slider.eventson = True
        self._update(theta_1_deg, theta_2_deg)
