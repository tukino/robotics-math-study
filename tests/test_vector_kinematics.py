"""Tests for rotation-matrix link vectors. / 回転行列によるリンクベクトルのテスト。"""

import unittest

import numpy as np

from robotics_math_study import (
    forward_kinematics_1link,
    forward_kinematics_2link,
    forward_kinematics_2link_vectors,
    link_vector_world,
)


class VectorKinematicsTests(unittest.TestCase):
    """Compare direct trigonometry with rotated local vectors.

    直接の三角関数と回転したローカルベクトルを比較します。
    """

    def test_one_link_matches_direct_formula_at_reference_angles(self) -> None:
        """Match both forms at 30°, 45°, and 90°. / 30°、45°、90°で両形式を一致させます。"""
        for theta_deg in (30.0, 45.0, 90.0):
            with self.subTest(theta_deg=theta_deg):
                direct = forward_kinematics_1link(5.0, theta_deg)
                rotated = link_vector_world(5.0, theta_deg)
                np.testing.assert_allclose(rotated, direct, atol=1e-12)

    def test_two_link_matches_direct_formula(self) -> None:
        """Match vector addition with the direct two-link formula.

        ベクトル加算と直接の2リンク式を一致させます。
        """
        for theta_1_deg, theta_2_deg in ((0.0, 0.0), (30.0, 45.0), (90.0, -45.0)):
            with self.subTest(theta_1_deg=theta_1_deg, theta_2_deg=theta_2_deg):
                direct = forward_kinematics_2link(5.0, 4.0, theta_1_deg, theta_2_deg)
                vectors = forward_kinematics_2link_vectors(
                    5.0, 4.0, theta_1_deg, theta_2_deg
                )
                np.testing.assert_allclose(vectors["p_elbow_world"], direct["elbow_world"])
                np.testing.assert_allclose(vectors["p_wrist_world"], direct["wrist_world"])
                np.testing.assert_allclose(
                    vectors["p_wrist_world"], vectors["v1_world"] + vectors["v2_world"]
                )

    def test_rejects_invalid_link_lengths(self) -> None:
        """Reject non-positive link lengths. / 正でないリンク長を拒否します。"""
        with self.assertRaises(ValueError):
            link_vector_world(0.0, 30.0)
        with self.assertRaises(ValueError):
            forward_kinematics_2link_vectors(5.0, -1.0, 30.0, 45.0)


if __name__ == "__main__":
    unittest.main()
