"""Tests for one-link forward kinematics. / 1リンク順運動学のテスト。"""

import math
import unittest

from robotics_math_study import forward_kinematics_1link


class ForwardKinematics1LinkTests(unittest.TestCase):
    """Verify endpoint calculations independently from Matplotlib.

    Matplotlibから独立して先端座標の計算を検証します。
    """

    def test_reference_angles_for_length_ten(self) -> None:
        """Check the 30°, 45°, and 90° learning examples.

        学習例である30°、45°、90°を確認します。
        """
        square_root_fifty = math.sqrt(50.0)
        cases = [
            (30.0, (5.0 * math.sqrt(3.0), 5.0)),
            (45.0, (square_root_fifty, square_root_fifty)),
            (90.0, (0.0, 10.0)),
        ]
        for theta_deg, expected in cases:
            with self.subTest(theta_deg=theta_deg):
                actual = forward_kinematics_1link(10.0, theta_deg)
                self.assertAlmostEqual(actual[0], expected[0])
                self.assertAlmostEqual(actual[1], expected[1])

    def test_angle_is_periodic(self) -> None:
        """Return the same endpoint after one full rotation.

        1回転後に同じ先端座標へ戻ることを確認します。
        """
        endpoint_at_zero = forward_kinematics_1link(10.0, 0.0)
        endpoint_at_full_turn = forward_kinematics_1link(10.0, 360.0)

        self.assertAlmostEqual(endpoint_at_zero[0], endpoint_at_full_turn[0])
        self.assertAlmostEqual(endpoint_at_zero[1], endpoint_at_full_turn[1])

    def test_rejects_invalid_link_lengths(self) -> None:
        """Reject non-positive or non-finite link lengths.

        正でない、または有限でないリンク長を拒否します。
        """
        for link_length in (0.0, -1.0, math.inf, math.nan):
            with self.subTest(link_length=link_length), self.assertRaises(ValueError):
                forward_kinematics_1link(link_length, 30.0)

    def test_rejects_non_finite_angles(self) -> None:
        """Reject angles that cannot describe a physical input.

        物理入力として扱えない角度を拒否します。
        """
        for theta_deg in (math.inf, -math.inf, math.nan):
            with self.subTest(theta_deg=theta_deg), self.assertRaises(ValueError):
                forward_kinematics_1link(10.0, theta_deg)


if __name__ == "__main__":
    unittest.main()
