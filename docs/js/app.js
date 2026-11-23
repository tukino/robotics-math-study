/**
 * ブラウザ教材のエントリポイント。
 */
(function () {
  "use strict";

  function init() {
    const canvas = document.getElementById("arm-canvas");
    const controlsRoot = document.getElementById("angle-controls");
    const exercisePanel = document.getElementById("exercise-panel");
    const verifyBox = document.getElementById("verify-box");

    const armView = new window.ArmView(canvas);
    armView.attachResize();

    const controller = new window.ExerciseController({
      armView,
      angleControls: null,
      panel: exercisePanel,
      linkLength1: window.LabConstants.LINK_LENGTH_1,
      linkLength2: window.LabConstants.LINK_LENGTH_2,
    });

    const angleControls = new window.AngleControls(controlsRoot, {
      theta1: 30,
      theta2: 45,
      onChange: (t1, t2) => controller.onAnglesChanged(t1, t2),
    });
    controller.angleControls = angleControls;

    function setActiveExercise(exerciseId) {
      document.querySelectorAll("[data-exercise]").forEach((b) => {
        b.classList.toggle("active", b.getAttribute("data-exercise") === exerciseId);
      });
      controller.start(exerciseId);
    }

    document.querySelectorAll("[data-exercise]").forEach((btn) => {
      btn.addEventListener("click", () => {
        setActiveExercise(btn.getAttribute("data-exercise"));
      });
    });

    renderVerification(verifyBox);
    setActiveExercise("predict");
  }

  function renderVerification(box) {
    const result = window.RoboticsKinematics.verifyAgainstPython(1e-9);
    const fmt = window.RoboticsKinematics.formatNumber;
    const rows = result.rows
      .map((row) => {
        return `<tr class="${row.pass ? "pass" : "fail"}">
          <td>Case ${row.name}</td>
          <td>${row.theta1}°, ${row.theta2}°</td>
          <td>(${fmt(row.elbow[0], 6)}, ${fmt(row.elbow[1], 6)})</td>
          <td>(${fmt(row.wrist[0], 6)}, ${fmt(row.wrist[1], 6)})</td>
          <td>${row.pass ? "一致" : "不一致"}</td>
        </tr>`;
      })
      .join("");

    box.innerHTML = `
      <h2>Python 版との照合（L₁=5, L₂=4）</h2>
      <p>ブラウザの順運動学が <code>forward_kinematics_2link</code> および
      <code>tests/test_two_link_kinematics.py</code> の代表姿勢と同じ値になることを確認します。
      演習画面のリンク長は見やすさのため 30 倍（150 mm / 120 mm）です。</p>
      <p class="verify-banner ${result.ok ? "ok" : "bad"}">
        ${result.ok ? "照合 OK：Case A〜D が Python 期待値と一致しました。" : "照合 NG：差分があります。"}
      </p>
      <div class="table-wrap">
        <table>
          <thead>
            <tr><th>Case</th><th>θ₁, θ₂</th><th>肘</th><th>手先</th><th>結果</th></tr>
          </thead>
          <tbody>${rows}</tbody>
        </table>
      </div>
    `;
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", init);
  } else {
    init();
  }
})();
