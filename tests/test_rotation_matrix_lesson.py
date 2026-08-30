"""Smoke tests for rotation-matrix visualizers. / 回転行列可視化のスモークテスト。"""

import unittest

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from robotics_math_study.rotation_matrix_lesson import (  # noqa: E402
    OneLinkRotationMatrixVisualizer,
    RulerViewComparisonVisualizer,
    TwoLinkRotationMatrixVisualizer,
)


class RotationMatrixVisualizerTests(unittest.TestCase):
    """Check that Slider-driven artists receive new coordinates.

    Slider駆動のArtistへ新しい座標が反映されることを確認します。
    """

    def tearDown(self) -> None:
        """Close figures after each test. / 各テスト後にFigureを閉じます。"""
        plt.close("all")

    def test_one_link_endpoint_updates(self) -> None:
        """Move the endpoint to (0, L) at 90°. / 90°で先端を(0, L)へ動かします。"""
        visualizer = OneLinkRotationMatrixVisualizer(5.0, 30.0)
        visualizer.set_angle(90.0)
        np.testing.assert_allclose(
            (visualizer.endpoint.get_xdata()[0], visualizer.endpoint.get_ydata()[0]),
            (0.0, 5.0),
            atol=1e-12,
        )

    def test_comparison_keeps_local_value_and_updates_desk_value(self) -> None:
        """Keep [L, 0] on the ruler while the desk endpoint moves.

        定規上では[L, 0]を保ちつつ、机上の先端を動かします。
        """
        visualizer = RulerViewComparisonVisualizer(5.0, 30.0)
        visualizer.set_angle(90.0)
        self.assertIn("[L, 0]", visualizer.local_invariant_text.get_text())
        self.assertTrue(
            any(
                "STATIC PREVIEW IN NOTEBOOKLM" in text.get_text()
                for text in visualizer.figure.texts
            )
        )
        np.testing.assert_allclose(
            (visualizer.world_endpoint.get_xdata()[0], visualizer.world_endpoint.get_ydata()[0]),
            (0.0, 5.0),
            atol=1e-12,
        )

    def test_two_link_joints_update(self) -> None:
        """Place both links on World Y at (90°, 0°). / (90°, 0°)で両リンクをWorld Y上へ置きます。"""
        visualizer = TwoLinkRotationMatrixVisualizer(5.0, 4.0, 30.0, 45.0)
        visualizer.set_angles(90.0, 0.0)
        np.testing.assert_allclose(visualizer.joints.get_xdata(), (0.0, 0.0, 0.0), atol=1e-12)
        np.testing.assert_allclose(visualizer.joints.get_ydata(), (0.0, 5.0, 9.0), atol=1e-12)


if __name__ == "__main__":
    unittest.main()
