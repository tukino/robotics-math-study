"""Tests for the coordinate-frame visualizer. / 座標系可視化のテスト。"""

import unittest

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt

from robotics_math_study.coordinate_frames import CoordinateFrameVisualizer


class CoordinateFrameVisualizerTests(unittest.TestCase):
    """Verify that rotating the frame updates both basis points.

    座標系の回転によって両方の基底点が更新されることを確認します。
    """

    def tearDown(self) -> None:
        """Close figures after each test.

        各テスト後に図を閉じます。
        """
        plt.close("all")

    def test_ninety_degrees_rotates_both_local_basis_points(self) -> None:
        """Move local x to world y and local y to negative world x.

        ローカルxを世界yへ、ローカルyを世界の負のxへ移動します。
        """
        visualizer = CoordinateFrameVisualizer(initial_angle_deg=0.0)

        visualizer.set_angle(90.0)

        p_x, p_y = visualizer.point_p_marker.get_data()
        q_x, q_y = visualizer.point_q_marker.get_data()
        self.assertAlmostEqual(p_x[0], 0.0)
        self.assertAlmostEqual(p_y[0], 1.0)
        self.assertAlmostEqual(q_x[0], -1.0)
        self.assertAlmostEqual(q_y[0], 0.0)
        self.assertIn("P local = (1, 0)", visualizer.info_text.get_text())
        self.assertIn("Q local = (0, 1)", visualizer.info_text.get_text())

    def test_creates_four_reference_angle_buttons(self) -> None:
        """Create buttons for 0°, 30°, 45°, and 90°.

        0°、30°、45°、90°のボタンを作成することを確認します。
        """
        visualizer = CoordinateFrameVisualizer()

        labels = [button.label.get_text() for button in visualizer.preset_buttons]
        self.assertEqual(labels, ["0°", "30°", "45°", "90°"])


if __name__ == "__main__":
    unittest.main()
