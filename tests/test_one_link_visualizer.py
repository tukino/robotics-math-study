"""Tests for the Matplotlib visualizer. / Matplotlib可視化のテスト。"""

import unittest

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt

from robotics_math_study.one_link_arm import OneLinkArmVisualizer


class OneLinkArmVisualizerTests(unittest.TestCase):
    """Verify that angle changes update all moving artists.

    角度変更によって動く描画要素が更新されることを確認します。
    """

    def tearDown(self) -> None:
        """Close figures after each test.

        各テスト後に図を閉じます。
        """
        plt.close("all")

    def test_set_angle_updates_endpoint_and_components(self) -> None:
        """Move the endpoint and component lines when the angle changes.

        角度変更時に先端と成分補助線が移動することを確認します。
        """
        visualizer = OneLinkArmVisualizer(link_length=10.0, initial_angle_deg=30.0)

        visualizer.set_angle(90.0)

        endpoint_x, endpoint_y = visualizer.endpoint_point.get_data()
        x_line_x, x_line_y = visualizer.x_component_line.get_data()
        y_line_x, y_line_y = visualizer.y_component_line.get_data()
        self.assertAlmostEqual(endpoint_x[0], 0.0)
        self.assertAlmostEqual(endpoint_y[0], 10.0)
        self.assertAlmostEqual(x_line_x[-1], 0.0)
        self.assertAlmostEqual(x_line_y[-1], 0.0)
        self.assertAlmostEqual(y_line_x[0], 0.0)
        self.assertAlmostEqual(y_line_y[-1], 10.0)
        self.assertIn("cos θ =  0.0000", visualizer.info_text.get_text())
        self.assertIn("sin θ =  1.0000", visualizer.info_text.get_text())

    def test_creates_three_preset_buttons(self) -> None:
        """Create buttons for 30°, 45°, and 90°.

        30°、45°、90°のボタンを作成することを確認します。
        """
        visualizer = OneLinkArmVisualizer()

        labels = [button.label.get_text() for button in visualizer.preset_buttons]
        self.assertEqual(labels, ["30°", "45°", "90°"])


if __name__ == "__main__":
    unittest.main()
