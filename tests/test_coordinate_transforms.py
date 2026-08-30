"""Tests for local/world coordinate transforms. / ローカル・世界座標変換のテスト。"""

import math
import unittest

import numpy as np

from robotics_math_study import local_to_world, rotation_matrix_2d


class CoordinateTransformTests(unittest.TestCase):
    """Verify the rotation matrix and transformed basis vectors.

    回転行列と変換後の基底ベクトルを検証します。
    """

    def test_local_x_at_reference_angles(self) -> None:
        """Check local (1, 0) at 0°, 30°, 45°, and 90°.

        ローカル(1, 0)を0°、30°、45°、90°で確認します。
        """
        square_root_half = math.sqrt(0.5)
        cases = [
            (0.0, (1.0, 0.0)),
            (30.0, (math.sqrt(3.0) / 2.0, 0.5)),
            (45.0, (square_root_half, square_root_half)),
            (90.0, (0.0, 1.0)),
        ]
        for theta_deg, expected in cases:
            with self.subTest(theta_deg=theta_deg):
                actual = local_to_world((1.0, 0.0), theta_deg)
                np.testing.assert_allclose(actual, expected, atol=1e-12)

    def test_matrix_columns_are_transformed_basis_vectors(self) -> None:
        """Match matrix columns with destinations of local x and y.

        行列の各列がローカルx、yの移動先と一致することを確認します。
        """
        rotation = rotation_matrix_2d(30.0)
        local_x_world = local_to_world((1.0, 0.0), 30.0)
        local_y_world = local_to_world((0.0, 1.0), 30.0)

        np.testing.assert_allclose(local_x_world, rotation[:, 0])
        np.testing.assert_allclose(local_y_world, rotation[:, 1])

    def test_rejects_invalid_inputs(self) -> None:
        """Reject malformed points and non-finite angles.

        不正な点と有限でない角度を拒否します。
        """
        with self.assertRaises(ValueError):
            local_to_world((1.0,), 30.0)
        with self.assertRaises(ValueError):
            local_to_world((1.0, math.nan), 30.0)
        with self.assertRaises(ValueError):
            rotation_matrix_2d(math.inf)


if __name__ == "__main__":
    unittest.main()
