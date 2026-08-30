"""Tests for one-link kinematics. / 1リンク運動学のテスト。"""

import math
import unittest
import xml.etree.ElementTree as element_tree

from robotics_math_study.one_link_arm import endpoint, render_svg


class EndpointTests(unittest.TestCase):
    """Verify endpoint calculations and input validation.

    手先位置の計算と入力検証を確認します。
    """

    def test_cardinal_angles(self) -> None:
        """Check endpoints at familiar unit-circle angles.

        単位円の代表的な角度で手先位置を確認します。
        """
        cases = [
            (0.0, (2.0, 0.0)),
            (math.pi / 2, (0.0, 2.0)),
            (math.pi, (-2.0, 0.0)),
        ]
        for angle, expected in cases:
            with self.subTest(angle=angle):
                actual = endpoint(angle, length=2.0)
                self.assertAlmostEqual(actual[0], expected[0])
                self.assertAlmostEqual(actual[1], expected[1])

    def test_rejects_non_finite_angles(self) -> None:
        """Reject angles that cannot represent a physical input.

        物理入力として扱えない角度を拒否することを確認します。
        """
        for angle in (math.inf, -math.inf, math.nan):
            with self.subTest(angle=angle), self.assertRaises(ValueError):
                endpoint(angle)

    def test_rejects_invalid_lengths(self) -> None:
        """Reject non-positive or non-finite link lengths.

        正でない、または有限でないリンク長を拒否することを確認します。
        """
        for length in (0.0, -1.0, math.inf, math.nan):
            with self.subTest(length=length), self.assertRaises(ValueError):
                endpoint(0.0, length)

    def test_svg_contains_endpoint_and_labels(self) -> None:
        """Include calculated coordinates and bilingual labels in the SVG.

        SVGに計算座標と日英のラベルが含まれることを確認します。
        """
        svg = render_svg(math.pi / 2, length=2.0)

        element_tree.fromstring(svg)
        self.assertIn("(0.000, 2.000)", svg)
        self.assertIn("One-link arm / 1リンクアーム", svg)
        self.assertTrue(svg.startswith("<svg"))


if __name__ == "__main__":
    unittest.main()
