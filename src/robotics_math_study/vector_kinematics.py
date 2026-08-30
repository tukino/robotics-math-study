"""Forward kinematics written as rotated link vectors. / 回転したリンクベクトルで書く順運動学。"""

from __future__ import annotations

import math
from typing import TypedDict

import numpy as np
from numpy.typing import NDArray

from .coordinate_transforms import rotation_matrix_2d


class TwoLinkVectorPose(TypedDict):
    """Local/World link vectors and joint positions for a two-link arm.

    2リンクアームのローカル・Worldリンクベクトルと関節位置です。
    """

    v1_local: NDArray[np.float64]
    v1_world: NDArray[np.float64]
    v2_local: NDArray[np.float64]
    v2_world: NDArray[np.float64]
    p_shoulder_world: NDArray[np.float64]
    p_elbow_world: NDArray[np.float64]
    p_wrist_world: NDArray[np.float64]
    theta1_world_deg: float
    theta2_world_deg: float


def link_vector_world(link_length: float, theta_world_deg: float) -> NDArray[np.float64]:
    """Rotate a link's fixed local vector ``[L, 0]`` into World coordinates.

    リンクに固定されたローカルベクトル ``[L, 0]`` をWorld座標へ回転します。
    """
    if not math.isfinite(link_length) or link_length <= 0:
        raise ValueError(
            "link_length must be positive and finite / "
            "リンク長は正の有限値にしてください"
        )

    # In its own frame, a link always points along local x, hence [L, 0].
    # The rotation matrix changes that same local vector into World components.
    # リンク自身の座標系では常にlocal x方向を向くため、ベクトルは[L, 0]です。
    # 回転行列はこの「ローカルでの同じベクトル」をWorldでの成分へ変換します。
    v_local = np.array([link_length, 0.0], dtype=float)
    return rotation_matrix_2d(theta_world_deg) @ v_local


def forward_kinematics_2link_vectors(
    link_length_1: float,
    link_length_2: float,
    theta_1_deg: float,
    theta_2_deg: float,
) -> TwoLinkVectorPose:
    """Compute two-link forward kinematics by rotating and adding vectors.

    各リンクベクトルを回転して足し、2リンク順運動学を計算します。

    ``theta_2_deg`` is relative to link 1, so link 2 has the World angle
    ``theta_1_deg + theta_2_deg``.

    ``theta_2_deg``は第1リンクに対する相対角度なので、第2リンクのWorld角度は
    ``theta_1_deg + theta_2_deg``です。
    """
    v1_local = np.array([link_length_1, 0.0], dtype=float)
    v2_local = np.array([link_length_2, 0.0], dtype=float)
    v1_world = link_vector_world(link_length_1, theta_1_deg)
    theta_2_world_deg = theta_1_deg + theta_2_deg
    v2_world = link_vector_world(link_length_2, theta_2_world_deg)

    p_shoulder_world = np.zeros(2, dtype=float)
    p_elbow_world = p_shoulder_world + v1_world
    p_wrist_world = p_elbow_world + v2_world

    return {
        "v1_local": v1_local,
        "v1_world": v1_world,
        "v2_local": v2_local,
        "v2_world": v2_world,
        "p_shoulder_world": p_shoulder_world,
        "p_elbow_world": p_elbow_world,
        "p_wrist_world": p_wrist_world,
        "theta1_world_deg": theta_1_deg,
        "theta2_world_deg": theta_2_world_deg,
    }
