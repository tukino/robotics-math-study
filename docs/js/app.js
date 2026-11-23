/**
 * ブラウザ教材のエントリポイント。
 */
(function () {
  "use strict";

  function syncNowDoingBarHeight(bar) {
    if (!bar) return;
    const heightPx = Math.ceil(bar.getBoundingClientRect().height);
    if (heightPx > 0) {
      document.documentElement.style.setProperty("--now-doing-bar-height", `${heightPx}px`);
    }
  }

  function init() {
    const canvas = document.getElementById("arm-canvas");
    const controlsRoot = document.getElementById("angle-controls");
    const exercisePanel = document.getElementById("exercise-panel");
    const verifyBox = document.getElementById("verify-box");
    const nowDoing = document.getElementById("now-doing");
    const nowDoingBar = document.querySelector(".now-doing-bar");
    const stageHint = document.getElementById("stage-hint");
    const controlsCaption = document.querySelector('[data-role="controls-caption"]');

    const armView = new window.ArmView(canvas);
    armView.attachResize();

    const controller = new window.ExerciseController({
      armView,
      angleControls: null,
      panel: exercisePanel,
      linkLength1: window.LabConstants.LINK_LENGTH_1,
      linkLength2: window.LabConstants.LINK_LENGTH_2,
      onProgress: ({ step, message, stageHint: hint, controlsHint }) => {
        updateProgress(step, message);
        if (hint && stageHint) stageHint.textContent = hint;
        if (controlsHint && controlsCaption) controlsCaption.textContent = controlsHint;
      },
      onRequestExercise: (id) => setActiveExercise(id),
    });

    const angleControls = new window.AngleControls(controlsRoot, {
      theta1: 30,
      theta2: 45,
      onChange: (t1, t2) => controller.onAnglesChanged(t1, t2),
    });
    controller.angleControls = angleControls;

    function updateProgress(step, message) {
      const order = ["look", "predict", "check", "correct", "compare"];
      const currentIndex = order.indexOf(step);
      document.querySelectorAll("#progress-bar li").forEach((li) => {
        const idx = order.indexOf(li.getAttribute("data-step"));
        li.classList.toggle("is-current", idx === currentIndex);
        li.classList.toggle("is-done", idx >= 0 && idx < currentIndex);
      });
      if (nowDoing) nowDoing.textContent = message;
      // 折り返しでバー高さが変わるので、次フレームで実測してオフセットを合わせる
      requestAnimationFrame(() => syncNowDoingBarHeight(nowDoingBar));
    }

    syncNowDoingBarHeight(nowDoingBar);
    window.addEventListener("resize", () => syncNowDoingBarHeight(nowDoingBar));

    let hasStarted = false;
    function setActiveExercise(exerciseId, { scroll = true } = {}) {
      controller.start(exerciseId);
      // 初回表示ではヘッダーを隠さない。演習切替時だけ手順へスクロールする。
      if (scroll && hasStarted) {
        exercisePanel.scrollIntoView({ behavior: "smooth", block: "start" });
      }
      hasStarted = true;
    }

    renderVerification(verifyBox.querySelector("[data-role='verify-body']") || verifyBox);
    setActiveExercise("predict", { scroll: false });
  }

  function renderVerification(box) {
    if (!box) return;
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
      <p>ブラウザ計算を、Python <code>forward_kinematics_2link</code> の固定数値（Case A〜D）と比較します。
      演習のリンク長は見やすさのため 30 倍（150 mm / 120 mm）です。</p>
      <p class="verify-banner ${result.ok ? "ok" : "bad"}">
        ${result.ok ? "照合 OK：Case A〜D が Python 固定期待値と一致しました。" : "照合 NG：差分があります。"}
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
