"""Forward-kinematics calculations. / 順運動学の座標計算。"""

from __future__ import annotations

import math
from typing import TypedDict


class TwoLinkPose(TypedDict):
    """World positions and absolute link angles for a two-link arm.

    2リンクアームのWorld位置とリンクの絶対角度です。
    """

    shoulder_world: tuple[float, float]
    elbow_world: tuple[float, float]
    wrist_world: tuple[float, float]
    shoulder_angle_world_deg: float
    forearm_angle_world_deg: float


def forward_kinematics_1link(link_length: float, theta_deg: float) -> tuple[float, float]:
    """Return the endpoint of a one-link planar arm.

    1リンク平面アームの先端座標を返します。

    The input angle is in degrees so that familiar angles such as 30°, 45°,
    and 90° are easy to use. Python's trigonometric functions expect radians,
    so the function converts the angle before calculating the coordinates.

    入力角は30°、45°、90°などを直感的に扱える度数法です。Pythonの三角関数は
    ラジアンを受け取るため、座標計算の前に角度を変換します。
    """
    if not math.isfinite(link_length) or link_length <= 0:
        raise ValueError(
            "link_length must be positive and finite / "
            "リンク長は正の有限値にしてください"
        )
    if not math.isfinite(theta_deg):
        raise ValueError("theta_deg must be finite / 有限の角度を指定してください")

    # math.cos() and math.sin() use radians, not degrees.
    # math.cos() と math.sin() は度ではなくラジアンを受け取ります。
    theta_rad = math.radians(theta_deg)

    # Rotating the unit vector (1, 0) by theta gives (cos(theta), sin(theta)).
    # 単位ベクトル (1, 0) をθだけ回すと (cosθ, sinθ) になります。
    cos_theta = math.cos(theta_rad)
    sin_theta = math.sin(theta_rad)

    # cos(theta) is the horizontal ratio, so L*cos(theta) is the x component.
    # cosθは水平方向の比なので、L*cosθがx方向成分になります。
    x = link_length * cos_theta

    # sin(theta) is the vertical ratio, so L*sin(theta) is the y component.
    # sinθは垂直方向の比なので、L*sinθがy方向成分になります。
    y = link_length * sin_theta

    # The endpoint is the vector from the joint at (0, 0) to (x, y).
    # 先端位置は、関節(0, 0)から(x, y)へ向かうベクトルそのものです。
    return x, y


def forward_kinematics_2link(
    link_length_1: float,
    link_length_2: float,
    theta_1_deg: float,
    theta_2_deg: float,
) -> TwoLinkPose:
    """Return shoulder, elbow, and wrist positions in the World frame.

    World座標系での肩、肘、手首位置を返します。

    ``theta_1_deg`` is the shoulder angle measured from World X.
    ``theta_2_deg`` is the elbow angle relative to the first link, so the
    forearm's absolute World angle is ``theta_1_deg + theta_2_deg``.

    ``theta_1_deg``はWorld Xから測る肩角度です。``theta_2_deg``は第1リンクに
    対する肘の相対角度なので、前腕のWorld絶対角度は
    ``theta_1_deg + theta_2_deg``です。
    """
    for name, link_length in (
        ("link_length_1", link_length_1),
        ("link_length_2", link_length_2),
    ):
        if not math.isfinite(link_length) or link_length <= 0:
            raise ValueError(
                f"{name} must be positive and finite / "
                "リンク長は正の有限値にしてください"
            )
    for name, theta_deg in (
        ("theta_1_deg", theta_1_deg),
        ("theta_2_deg", theta_2_deg),
    ):
        if not math.isfinite(theta_deg):
            raise ValueError(f"{name} must be finite / 有限の角度を指定してください")

    theta_1_rad = math.radians(theta_1_deg)
    forearm_angle_world_deg = theta_1_deg + theta_2_deg
    forearm_angle_world_rad = math.radians(forearm_angle_world_deg)

    # Shoulder -> elbow vector, expressed in the World frame.
    # 肩→肘ベクトルをWorld座標系で表します。
    x_elbow = link_length_1 * math.cos(theta_1_rad)
    y_elbow = link_length_1 * math.sin(theta_1_rad)

    # Elbow -> wrist also needs a World angle. Because theta_2 is relative to
    # link 1, its World angle is theta_1 + theta_2.
    # 肘→手首もWorld角度が必要です。theta_2は第1リンクに対する相対角度なので、
    # World角度はtheta_1 + theta_2になります。
    x_forearm = link_length_2 * math.cos(forearm_angle_world_rad)
    y_forearm = link_length_2 * math.sin(forearm_angle_world_rad)

    # Wrist = (shoulder -> elbow) + (elbow -> wrist), both expressed in World.
    # 手首位置は、World表現に揃えた「肩→肘」と「肘→手首」のベクトル和です。
    x_wrist = x_elbow + x_forearm
    y_wrist = y_elbow + y_forearm

    return {
        "shoulder_world": (0.0, 0.0),
        "elbow_world": (x_elbow, y_elbow),
        "wrist_world": (x_wrist, y_wrist),
        "shoulder_angle_world_deg": theta_1_deg,
        "forearm_angle_world_deg": forearm_angle_world_deg,
    }
