/**
 * Node から順運動学の照合と演習2シナリオの数値を確認する。
 * 使い方: node docs/verify.cjs
 */
require("./js/kinematics.js");
const K = globalThis.RoboticsKinematics;

const verify = K.verifyAgainstPython(1e-9);
if (!verify.ok) {
  console.error("Python cross-check FAILED");
  console.error(verify.rows);
  process.exit(1);
}
console.log("Python cross-check OK (Cases A–D, L1=5, L2=4)");

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
