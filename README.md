# Robotics Math Study

[日本語版](README.ja.md)

A small, reproducible lab for learning the mathematics behind robotics through
explanations, code, visualizations, and retrieval practice.

This repository is not a chronological study diary. Each topic should become an
artifact that another learner can run, inspect, and explain.

## Learning path

1. Trigonometry and the unit circle
2. Vectors and coordinate components
3. Rotation matrices
4. One-link forward kinematics
5. Local and World coordinate frames
6. Multiple Origins and two-link forward kinematics

## First artifact: one-link planar arm

For a link of length $L$ and a joint angle $\theta$, the endpoint is

$$
x = L\cos(\theta), \qquad y = L\sin(\theta).
$$

The included Python module calculates this endpoint and visualizes the arm with
Matplotlib. A 0–360° Slider updates the arm, endpoint, component lines, and
numeric values in real time. Buttons jump directly to 30°, 45°, and 90°.

```text
angle θ -> sin/cos components -> endpoint (x, y) -> visualization
```

## Quick start

Python 3.11 or later is recommended.

```bash
python -m venv .venv
python -m pip install -e .
python -m robotics_math_study.one_link_arm
python examples/verify_one_link_angles.py
python -m unittest discover -s tests
```

Activate the virtual environment before installing if you want to keep the
dependencies isolated. On PowerShell, run `.venv\Scripts\Activate.ps1`; on
macOS or Linux, run `source .venv/bin/activate`.

The first command opens the interactive Matplotlib window. The verification
script prints the endpoints for 30°, 45°, and 90° so that they can be compared
with the figure.

For the interactive Jupyter Notebook:

```bash
python -m pip install -e ".[notebook]"
jupyter lab notebooks/01_one_link_forward_kinematics.ipynb
```

In the Notebook, `visualizer.set_angle(30)`, `visualizer.set_angle(45)`, and
`visualizer.set_angle(90)` provide the same preset-angle behavior as the buttons.

## Where the forward kinematics is

`forward_kinematics_1link()` is the forward-kinematics calculation. It receives
the known link length and joint angle. It converts them into the endpoint
`(x, y)` using `L*cos(θ)` and `L*sin(θ)`. The Matplotlib code only visualizes
that result and updates the artists when the angle changes.

## Second artifact: local and world coordinate frames

The coordinate-frame lesson keeps the world axes `X, Y` fixed while the local
axes `x', y'` rotate with the arm. It transforms the local basis points with
`P_world = R(theta) @ P_local` and displays both local and world coordinates.

```bash
python examples/visualize_local_world_frames.py
```

The Slider and preset buttons show 0°, 30°, 45°, and 90°. The red point is the
world destination of local `(1, 0)`, which is the first column of the rotation
matrix. The green point is the destination of local `(0, 1)`, which is the
second column.

The same lesson is available as a Notebook:

```bash
jupyter lab notebooks/02_local_and_world_frames.ipynb
```

## Third artifact: multiple Origins on a two-link arm

The two-link lesson compares one fixed World frame with shoulder and elbow
local frames. The elbow remains `(0, 0)` in its own frame while its World
position changes with `theta1`. The forearm angle in World is the relative
angle sum `theta1 + theta2`.

```bash
python -m robotics_math_study.two_link_frames
jupyter lab notebooks/03_multiple_origins_and_frames.ipynb
```

Two Sliders update both links, both local frames, elbow and wrist World
coordinates, and the vector addition used by two-link forward kinematics.
Cases A through D provide the representative angle combinations used in the
Notebook questions.

## Repository structure

```text
robotics-math-study/
├── examples/                  # Numerical checks and interactive lessons
├── notebooks/                 # Interactive, executable experiments
├── notes/                     # Explanations written in our own words
├── src/robotics_math_study/   # Kinematics and visualization modules
└── tests/                     # Small checks for mathematical behavior
```

## Publishing policy

Appropriate public content includes original explanations, code,
visualizations, exercises, and reflections. Do not commit copied textbook or
course materials, assignment prompts or submitted answers, confidential work
information, customer information, credentials, personal data, or generated
files that contain them.

When a topic comes from a class or book, restate the concept independently and
cite the source instead of reproducing its protected content.

## Working method

For each topic:

1. Explain the idea in plain language.
2. Solve a small number of representative exercises.
3. Implement or visualize the idea.
4. Add a test or a retrieval question.
5. Record sources and remaining questions.

## License

No license has been selected yet. Copyright remains with the repository owner
unless a license is added later.
