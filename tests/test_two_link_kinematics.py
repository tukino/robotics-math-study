"""Tests for two-link forward kinematics. / 2リンク順運動学のテスト。"""

import math
import unittest

import numpy as np

from robotics_math_study import forward_kinematics_2link, frame_axes_world


class ForwardKinematics2LinkTests(unittest.TestCase):
    """Verify joint positions, relative angles, and frame directions.

    関節位置、相対角度、座標軸方向を検証します。
    """

    def test_case_a_both_links_point_along_world_x(self) -> None:
        """Place elbow and wrist on World X for theta1=0°, theta2=0°.

        theta1=0°、theta2=0°で肘と手首をWorld X上へ配置します。
        """
        pose = forward_kinematics_2link(5.0, 4.0, 0.0, 0.0)

        np.testing.assert_allclose(pose["shoulder_world"], (0.0, 0.0))
        np.testing.assert_allclose(pose["elbow_world"], (5.0, 0.0))
        np.testing.assert_allclose(pose["wrist_world"], (9.0, 0.0))

    def test_case_c_uses_theta1_plus_theta2_for_forearm(self) -> None:
        """Use 75° as the World forearm angle for theta1=30°, theta2=45°.

        theta1=30°、theta2=45°で前腕のWorld角度を75°として計算します。
        """
        pose = forward_kinematics_2link(5.0, 4.0, 30.0, 45.0)
        expected_elbow = (
            5.0 * math.cos(math.radians(30.0)),
            5.0 * math.sin(math.radians(30.0)),
        )
        expected_wrist = (
            expected_elbow[0] + 4.0 * math.cos(math.radians(75.0)),
            expected_elbow[1] + 4.0 * math.sin(math.radians(75.0)),
        )

        np.testing.assert_allclose(pose["elbow_world"], expected_elbow)
        np.testing.assert_allclose(pose["wrist_world"], expected_wrist)
        self.assertEqual(pose["forearm_angle_world_deg"], 75.0)

    def test_frame_axes_follow_their_world_angles(self) -> None:
        """Rotate shoulder and elbow axes to their absolute World angles.

        肩と肘の座標軸を、それぞれのWorld絶対角度へ回転します。
        """
        shoulder_x, shoulder_y = frame_axes_world(30.0)
        elbow_x, elbow_y = frame_axes_world(75.0)

        np.testing.assert_allclose(
            shoulder_x,
            (math.cos(math.radians(30.0)), math.sin(math.radians(30.0))),
        )
        np.testing.assert_allclose(
            shoulder_y,
            (-math.sin(math.radians(30.0)), math.cos(math.radians(30.0))),
        )
        np.testing.assert_allclose(
            elbow_x,
            (math.cos(math.radians(75.0)), math.sin(math.radians(75.0))),
        )
        np.testing.assert_allclose(
            elbow_y,
            (-math.sin(math.radians(75.0)), math.cos(math.radians(75.0))),
        )

    def test_rejects_invalid_inputs(self) -> None:
        """Reject invalid lengths and non-finite joint angles.

        不正なリンク長と有限でない関節角度を拒否します。
        """
        invalid_arguments = [
            (0.0, 4.0, 0.0, 0.0),
            (5.0, -1.0, 0.0, 0.0),
            (5.0, 4.0, math.nan, 0.0),
            (5.0, 4.0, 0.0, math.inf),
        ]
        for arguments in invalid_arguments:
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                forward_kinematics_2link(*arguments)


if __name__ == "__main__":
    unittest.main()
