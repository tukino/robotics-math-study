"""Print reference one-link endpoints. / 代表角度の1リンク先端座標を表示。"""

import math

from robotics_math_study import forward_kinematics_1link


def main() -> None:
    """Print x and y for 30°, 45°, and 90° with a link length of 10.

    リンク長10について、30°、45°、90°のxとyを表示します。
    """
    link_length = 10.0
    for theta_deg in (30.0, 45.0, 90.0):
        x, y = forward_kinematics_1link(link_length, theta_deg)
        theta_rad = math.radians(theta_deg)
        print(
            f"θ={theta_deg:>5.1f}°  "
            f"cosθ={math.cos(theta_rad): .4f}  "
            f"sinθ={math.sin(theta_rad): .4f}  "
            f"endpoint=({x: .4f}, {y: .4f})"
        )


if __name__ == "__main__":
    main()
