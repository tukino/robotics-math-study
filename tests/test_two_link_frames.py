"""Tests for the two-link frame visualizer. / 2リンク座標系可視化のテスト。"""

import unittest

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt

from robotics_math_study.two_link_frames import TwoLinkFrameVisualizer


class TwoLinkFrameVisualizerTests(unittest.TestCase):
    """Verify updates for multiple Origins and learning cases.

    複数Originと学習ケースの更新を検証します。
    """

    def tearDown(self) -> None:
        """Close figures after each test.

        各テスト後に図を閉じます。
        """
        plt.close("all")

    def test_case_b_moves_elbow_world_but_keeps_elbow_local_zero(self) -> None:
        """Move the elbow in World while its elbow-local coordinate stays zero.

        肘ローカル座標をゼロに保ったまま、World上の肘を移動します。
        """
        visualizer = TwoLinkFrameVisualizer()

        visualizer.set_case("A")
        elbow_a_x, elbow_a_y = visualizer.elbow_marker.get_data()
        visualizer.set_case("B")
        elbow_b_x, elbow_b_y = visualizer.elbow_marker.get_data()

        self.assertAlmostEqual(elbow_a_x[0], 5.0)
        self.assertAlmostEqual(elbow_a_y[0], 0.0)
        self.assertAlmostEqual(elbow_b_x[0], 5.0 * 3.0**0.5 / 2.0)
        self.assertAlmostEqual(elbow_b_y[0], 2.5)
        self.assertIn("Elbow local: (0,0)", visualizer.elbow_text.get_text())
        self.assertIn("Elbow in World", visualizer.elbow_text.get_text())

    def test_case_c_displays_seventy_five_degree_forearm_angle(self) -> None:
        """Display theta1 + theta2 as 75° for Case C.

        Case Cでtheta1 + theta2を75°として表示します。
        """
        visualizer = TwoLinkFrameVisualizer()

        visualizer.set_case("C")

        self.assertEqual(visualizer.theta_1_slider.val, 30.0)
        self.assertEqual(visualizer.theta_2_slider.val, 45.0)
        self.assertIn("=    75.0°", visualizer.info_text.get_text())

    def test_creates_four_case_buttons(self) -> None:
        """Create buttons for Cases A through D.

        Case A〜Dのボタンを作成することを確認します。
        """
        visualizer = TwoLinkFrameVisualizer()

        labels = [button.label.get_text() for button in visualizer.case_buttons]
        self.assertEqual(
            labels,
            ["Case A\n(0°, 0°)", "Case B\n(30°, 0°)", "Case C\n(30°, 45°)", "Case D\n(90°, -45°)"],
        )


if __name__ == "__main__":
    unittest.main()
