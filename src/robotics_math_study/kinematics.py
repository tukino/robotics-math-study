"""Forward-kinematics calculations. / 順運動学の座標計算。"""

from __future__ import annotations

import math


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
