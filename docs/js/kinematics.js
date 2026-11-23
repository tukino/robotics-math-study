/**
 * 2リンク平面アームの順運動学・ヤコビ行列・一次補正。
 * Python版 robotics_math_study.kinematics.forward_kinematics_2link と同じ式。
 *
 * θ1: World X から測る肩角度（度）
 * θ2: 第1リンクに対する肘の相対角度（度）
 * 前腕の World 絶対角度 = θ1 + θ2
 */
(function (global) {
  "use strict";

  const DEG2RAD = Math.PI / 180;
  const RAD2DEG = 180 / Math.PI;

  /**
   * @typedef {{
   *   shoulder_world: [number, number],
   *   elbow_world: [number, number],
   *   wrist_world: [number, number],
   *   shoulder_angle_world_deg: number,
   *   forearm_angle_world_deg: number
   * }} TwoLinkPose
   */

  /**
   * @param {number} linkLength1
   * @param {number} linkLength2
   * @param {number} theta1Deg
   * @param {number} theta2Deg
   * @returns {TwoLinkPose}
   */
  function forwardKinematics2Link(linkLength1, linkLength2, theta1Deg, theta2Deg) {
    if (!(linkLength1 > 0) || !Number.isFinite(linkLength1)) {
      throw new Error("linkLength1 must be positive and finite");
    }
    if (!(linkLength2 > 0) || !Number.isFinite(linkLength2)) {
      throw new Error("linkLength2 must be positive and finite");
    }
    if (!Number.isFinite(theta1Deg) || !Number.isFinite(theta2Deg)) {
      throw new Error("joint angles must be finite");
    }

    const theta1Rad = theta1Deg * DEG2RAD;
    const forearmAngleWorldDeg = theta1Deg + theta2Deg;
    const forearmAngleWorldRad = forearmAngleWorldDeg * DEG2RAD;

    const xElbow = linkLength1 * Math.cos(theta1Rad);
    const yElbow = linkLength1 * Math.sin(theta1Rad);
    const xForearm = linkLength2 * Math.cos(forearmAngleWorldRad);
    const yForearm = linkLength2 * Math.sin(forearmAngleWorldRad);

    return {
      shoulder_world: [0, 0],
      elbow_world: [xElbow, yElbow],
      wrist_world: [xElbow + xForearm, yElbow + yForearm],
      shoulder_angle_world_deg: theta1Deg,
      forearm_angle_world_deg: forearmAngleWorldDeg,
    };
  }

  /**
   * ヤコビ行列 J（ラジアン角微分、単位は長さ/rad）。
   * ∂(x,y)/∂(θ1_rad, θ2_rad)
   * @returns {[[number, number], [number, number]]}
   */
  function jacobian2LinkRad(linkLength1, linkLength2, theta1Deg, theta2Deg) {
    const t1 = theta1Deg * DEG2RAD;
    const t12 = (theta1Deg + theta2Deg) * DEG2RAD;
    const s1 = Math.sin(t1);
    const c1 = Math.cos(t1);
    const s12 = Math.sin(t12);
    const c12 = Math.cos(t12);
    return [
      [-linkLength1 * s1 - linkLength2 * s12, -linkLength2 * s12],
      [linkLength1 * c1 + linkLength2 * c12, linkLength2 * c12],
    ];
  }

  /** 2x2 行列式 */
  function det2(m) {
    return m[0][0] * m[1][1] - m[0][1] * m[1][0];
  }

  /**
   * 最小二乗で J * dθ_rad = [dx, dy] を解く（特異時は擬似逆）。
   * @returns {{
   *   dTheta1Deg: number,
   *   dTheta2Deg: number,
   *   singular: boolean,
   *   det: number,
   *   predictedDelta: [number, number]
   * }}
   */
  function solveJointCorrection(linkLength1, linkLength2, theta1Deg, theta2Deg, dx, dy) {
    const J = jacobian2LinkRad(linkLength1, linkLength2, theta1Deg, theta2Deg);
    const determinant = det2(J);
    const singular = Math.abs(determinant) < 1e-8;

    let dThetaRad;
    if (!singular) {
      const invDet = 1 / determinant;
      dThetaRad = [
        invDet * (J[1][1] * dx - J[0][1] * dy),
        invDet * (-J[1][0] * dx + J[0][0] * dy),
      ];
    } else {
      // ランク落ち: 正規方程式 J^T J dθ = J^T δ を対角に微小減衰して解く
      const jt00 = J[0][0] * J[0][0] + J[1][0] * J[1][0];
      const jt01 = J[0][0] * J[0][1] + J[1][0] * J[1][1];
      const jt11 = J[0][1] * J[0][1] + J[1][1] * J[1][1];
      const rhs0 = J[0][0] * dx + J[1][0] * dy;
      const rhs1 = J[0][1] * dx + J[1][1] * dy;
      const lambda = 1e-9;
      const a00 = jt00 + lambda;
      const a11 = jt11 + lambda;
      const a01 = jt01;
      const d = a00 * a11 - a01 * a01;
      if (Math.abs(d) < 1e-18) {
        dThetaRad = [0, 0];
      } else {
        dThetaRad = [
          (a11 * rhs0 - a01 * rhs1) / d,
          (-a01 * rhs0 + a00 * rhs1) / d,
        ];
      }
    }

    const predictedDelta = [
      J[0][0] * dThetaRad[0] + J[0][1] * dThetaRad[1],
      J[1][0] * dThetaRad[0] + J[1][1] * dThetaRad[1],
    ];

    return {
      dTheta1Deg: dThetaRad[0] * RAD2DEG,
      dTheta2Deg: dThetaRad[1] * RAD2DEG,
      singular,
      det: determinant,
      predictedDelta,
    };
  }

  /**
   * 手先目標が2リンク平面アームの作業領域内か。
   * |L1 - L2| ≤ r ≤ L1 + L2
   */
  function isReachable(linkLength1, linkLength2, x, y) {
    const r = Math.hypot(x, y);
    const rMin = Math.abs(linkLength1 - linkLength2);
    const rMax = linkLength1 + linkLength2;
    return r >= rMin - 1e-9 && r <= rMax + 1e-9;
  }

  /**
   * Python `forward_kinematics_2link(5, 4, …)` の出力を固定した照合用期待値。
   * JS 側で三角関数を再計算しない（式を同時に壊しても検出できるようにする）。
   * 再生成: python3 -c "from robotics_math_study.kinematics import forward_kinematics_2link; …"
   */
  const PYTHON_REFERENCE_CASES = {
    A: { theta1: 0, theta2: 0, elbow: [5.0, 0.0], wrist: [9.0, 0.0] },
    B: {
      theta1: 30,
      theta2: 0,
      elbow: [4.330127018922194, 2.4999999999999996],
      wrist: [7.794228634059948, 4.499999999999999],
    },
    C: {
      theta1: 30,
      theta2: 45,
      elbow: [4.330127018922194, 2.4999999999999996],
      wrist: [5.365403199332277, 6.363703305156273],
    },
    D: {
      theta1: 90,
      theta2: -45,
      elbow: [3.061616997868383e-16, 5.0],
      wrist: [2.8284271247461907, 7.82842712474619],
    },
  };

  /**
   * L1=5, L2=4 の代表姿勢で JS 計算が Python 固定期待値と一致するか検証する。
   * @returns {{ ok: boolean, rows: Array<object> }}
   */
  function verifyAgainstPython(tolerance = 1e-9) {
    const rows = [];
    let ok = true;
    for (const [name, expected] of Object.entries(PYTHON_REFERENCE_CASES)) {
      const pose = forwardKinematics2Link(5, 4, expected.theta1, expected.theta2);
      const elbowErr = Math.hypot(
        pose.elbow_world[0] - expected.elbow[0],
        pose.elbow_world[1] - expected.elbow[1]
      );
      const wristErr = Math.hypot(
        pose.wrist_world[0] - expected.wrist[0],
        pose.wrist_world[1] - expected.wrist[1]
      );
      const pass = elbowErr <= tolerance && wristErr <= tolerance;
      if (!pass) ok = false;
      rows.push({
        name,
        theta1: expected.theta1,
        theta2: expected.theta2,
        elbow: pose.elbow_world,
        wrist: pose.wrist_world,
        elbowErr,
        wristErr,
        pass,
      });
    }
    return { ok, rows };
  }

  function formatNumber(value, digits = 3) {
    if (!Number.isFinite(value)) return "—";
    const fixed = value.toFixed(digits);
    return Object.is(value, -0) || fixed === "-0.000" || fixed === "-0.00" || fixed === "-0.0"
      ? (0).toFixed(digits)
      : fixed;
  }

  global.RoboticsKinematics = {
    forwardKinematics2Link,
    jacobian2LinkRad,
    solveJointCorrection,
    isReachable,
    verifyAgainstPython,
    formatNumber,
    PYTHON_REFERENCE_CASES,
    DEG2RAD,
    RAD2DEG,
  };
})(typeof window !== "undefined" ? window : globalThis);
