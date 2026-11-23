/**
 * Node から順運動学の照合と演習2シナリオの数値を確認する。
 * 使い方: node docs/verify.cjs
 *
 * 可能なら Python の forward_kinematics_2link を実行し、JS 固定期待値とも照合する。
 */
const { spawnSync } = require("child_process");
const path = require("path");

require("./js/kinematics.js");
const K = globalThis.RoboticsKinematics;

const verify = K.verifyAgainstPython(1e-9);
if (!verify.ok) {
  console.error("Python fixed-constant cross-check FAILED");
  console.error(verify.rows);
  process.exit(1);
}
console.log("JS vs fixed Python constants OK (Cases A–D, L1=5, L2=4)");

function runLivePythonCheck() {
  const script = `
from robotics_math_study.kinematics import forward_kinematics_2link
import json
cases = {"A": (0,0), "B": (30,0), "C": (30,45), "D": (90,-45)}
out = {}
for name, (t1, t2) in cases.items():
    p = forward_kinematics_2link(5, 4, t1, t2)
    out[name] = {
        "elbow": list(p["elbow_world"]),
        "wrist": list(p["wrist_world"]),
    }
print(json.dumps(out))
`;
  const result = spawnSync("python3", ["-c", script], {
    cwd: path.resolve(__dirname, ".."),
    encoding: "utf8",
  });
  if (result.status !== 0) {
    console.warn("Live Python check skipped:", (result.stderr || result.error || "").toString().trim());
    return;
  }
  const live = JSON.parse(result.stdout);
  for (const [name, expected] of Object.entries(K.PYTHON_REFERENCE_CASES)) {
    const pose = K.forwardKinematics2Link(5, 4, expected.theta1, expected.theta2);
    const py = live[name];
    const elbowErr = Math.hypot(pose.elbow_world[0] - py.elbow[0], pose.elbow_world[1] - py.elbow[1]);
    const wristErr = Math.hypot(pose.wrist_world[0] - py.wrist[0], pose.wrist_world[1] - py.wrist[1]);
    const constElbowErr = Math.hypot(expected.elbow[0] - py.elbow[0], expected.elbow[1] - py.elbow[1]);
    const constWristErr = Math.hypot(expected.wrist[0] - py.wrist[0], expected.wrist[1] - py.wrist[1]);
    if (elbowErr > 1e-9 || wristErr > 1e-9 || constElbowErr > 1e-9 || constWristErr > 1e-9) {
      console.error("Live Python mismatch on", name, { elbowErr, wristErr, constElbowErr, constWristErr });
      process.exit(1);
    }
  }
  console.log("Live Python forward_kinematics_2link OK");
}

runLivePythonCheck();

const L1 = 150;
const L2 = 120;
const scenarios = [
  {
    id: "bent",
    t1: 30,
    t2: 45,
    dx: 5,
    dy: 0,
    expectSingular: false,
    expectReachable: true,
    maxResid: 1,
  },
  {
    id: "extended-in",
    t1: 0,
    t2: 0,
    dx: -5,
    dy: 0,
    expectSingular: true,
    expectReachable: true,
    minResid: 4.5,
  },
  {
    id: "extended-out",
    t1: 0,
    t2: 0,
    dx: 5,
    dy: 0,
    expectSingular: true,
    expectReachable: false,
    minResid: 4.5,
  },
];

for (const sc of scenarios) {
  const start = K.forwardKinematics2Link(L1, L2, sc.t1, sc.t2).wrist_world;
  const target = [start[0] + sc.dx, start[1] + sc.dy];
  const sol = K.solveJointCorrection(L1, L2, sc.t1, sc.t2, sc.dx, sc.dy);
  const after = K.forwardKinematics2Link(
    L1,
    L2,
    sc.t1 + sol.dTheta1Deg,
    sc.t2 + sol.dTheta2Deg
  ).wrist_world;
  const resid = Math.hypot(after[0] - target[0], after[1] - target[1]);
  const reachable = K.isReachable(L1, L2, target[0], target[1]);

  if (sol.singular !== sc.expectSingular) {
    console.error(sc.id, "singular mismatch", sol.singular);
    process.exit(1);
  }
  if (reachable !== sc.expectReachable) {
    console.error(sc.id, "reachable mismatch", reachable);
    process.exit(1);
  }
  if (sc.maxResid != null && resid > sc.maxResid) {
    console.error(sc.id, "residual too large", resid);
    process.exit(1);
  }
  if (sc.minResid != null && resid < sc.minResid) {
    console.error(sc.id, "residual unexpectedly small", resid);
    process.exit(1);
  }
  console.log(
    `${sc.id}: resid=${resid.toFixed(4)} mm, singular=${sol.singular}, reachable=${reachable}`
  );
}

console.log("All scenario checks passed.");
