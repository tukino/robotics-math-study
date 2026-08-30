"""Run the local/world coordinate-frame lesson. / ローカル・世界座標系の学習例。"""

import numpy as np

from robotics_math_study import rotation_matrix_2d
from robotics_math_study.coordinate_frames import CoordinateFrameVisualizer


def print_reference_angles() -> None:
    """Print world destinations of the two local basis vectors.

    2つのローカル基底ベクトルの世界座標での移動先を表示します。
    """
    local_x = np.array([1.0, 0.0])
    local_y = np.array([0.0, 1.0])

    for theta_deg in (0.0, 30.0, 45.0, 90.0):
        rotation = rotation_matrix_2d(theta_deg)
        x_axis_world = rotation @ local_x
        y_axis_world = rotation @ local_y
        print(
            f"θ={theta_deg:>5.1f}°  "
            f"local (1,0) -> world ({x_axis_world[0]: .4f}, {x_axis_world[1]: .4f})  "
            f"local (0,1) -> world ({y_axis_world[0]: .4f}, {y_axis_world[1]: .4f})"
        )


def main() -> None:
    """Print reference values and launch the interactive figure.

    代表角度の値を表示し、対話的な図を起動します。
    """
    print_reference_angles()
    visualizer = CoordinateFrameVisualizer(initial_angle_deg=0.0)
    visualizer.show()


if __name__ == "__main__":
    main()
