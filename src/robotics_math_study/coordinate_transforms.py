"""Two-dimensional coordinate-frame transformations. / 2次元座標系の変換。"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np
from numpy.typing import NDArray


def rotation_matrix_2d(theta_deg: float) -> NDArray[np.float64]:
    """Return the 2D rotation matrix for an angle in degrees.

    度数法の角度に対応する2次元回転行列を返します。
    """
    if not math.isfinite(theta_deg):
        raise ValueError("theta_deg must be finite / 有限の角度を指定してください")

    # NumPy's sin/cos functions expect radians, so convert the learner-friendly
    # degree input before constructing the matrix.
    # NumPyのsin/cosはラジアンを受け取るため、学習しやすい度数法の入力を
    # 行列の作成前に変換します。
    theta_rad = math.radians(theta_deg)
    cos_theta = math.cos(theta_rad)
    sin_theta = math.sin(theta_rad)

    return np.array(
        [
            [cos_theta, -sin_theta],
            [sin_theta, cos_theta],
        ],
        dtype=float,
    )


def local_to_world(
    point_local: Sequence[float] | NDArray[np.float64],
    theta_deg: float,
) -> NDArray[np.float64]:
    """Rotate a point from the local frame into the world frame.

    点をローカル座標系から世界座標系へ回転変換します。
    """
    point = np.asarray(point_local, dtype=float)
    if point.shape != (2,) or not np.all(np.isfinite(point)):
        raise ValueError(
            "point_local must contain two finite values / "
            "ローカル座標は2つの有限値にしてください"
        )

    rotation = rotation_matrix_2d(theta_deg)

    # Matrix-vector multiplication forms a weighted sum of the matrix columns.
    # R @ (1, 0) selects 1 * column 1 + 0 * column 2, so the destination of the
    # local x basis vector is exactly the first column of R.
    # 行列とベクトルの積は、行列の各列の重み付き和です。
    # R @ (1, 0) は「第1列×1 + 第2列×0」なので、ローカルx基底の移動先が
    # そのままRの第1列になります。

    # Likewise, R @ (0, 1) selects 0 * column 1 + 1 * column 2. Therefore the
    # destination of the local y basis vector is the second column of R.
    # 同様にR @ (0, 1)は「第1列×0 + 第2列×1」なので、ローカルy基底の
    # 移動先がRの第2列になります。
    point_world = rotation @ point
    return point_world


def frame_axes_world(theta_world_deg: float) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Return local x and y unit-axis directions expressed in World.

    ローカルx、y単位軸の向きをWorld座標で返します。
    """
    rotation = rotation_matrix_2d(theta_world_deg)
    local_x = np.array([1.0, 0.0])
    local_y = np.array([0.0, 1.0])
    return rotation @ local_x, rotation @ local_y
