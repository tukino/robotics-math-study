/**
 * 演習定義と演習コントローラ。
 * 図・操作・計算は ArmView / AngleControls / RoboticsKinematics を再利用する。
 */
(function (global) {
  "use strict";

  const LINK_LENGTH_1 = 150; // mm（Python の L1=5 と比 5:4 を保つ）
  const LINK_LENGTH_2 = 120; // mm

  /** @type {Record<string, {id: string, title: string, summary: string}>} */
  const EXERCISE_META = {
    predict: {
      id: "predict",
      title: "演習1：角度から手先位置を予想する",
      summary: "肩と肘の角度を見て手先座標を予想し、スライダーで確かめる。",
    },
    correct: {
      id: "correct",
      title: "演習2：5 mm 目標と一次補正",
      summary: "手先を 5 mm 動かす目標に対し、関節角を1回だけ補正して残差を見る。",
    },
  };

  /**
   * 演習2のシナリオ。
   * bent: 肘を曲げた姿勢（一次近似がよく効く）
   * extended-in: 伸ばし切り＋内側（到達可能だが特異で一次近似が効かない）
   * extended-out: 伸ばし切り＋外側（到達不能）
   */
  const CORRECTION_SCENARIOS = {
    bent: {
      id: "bent",
      label: "肘を曲げた姿勢",
      theta1: 30,
      theta2: 45,
      delta: [5, 0],
      hint: "θ₁=30°, θ₂=45°。手先は作業領域の内側にあり、ヤコビ行列は正則です。",
    },
    "extended-in": {
      id: "extended-in",
      label: "伸ばし切り・内側（到達可能）",
      theta1: 0,
      theta2: 0,
      delta: [-5, 0],
      hint: "θ₁=0°, θ₂=0°（特異姿勢）。目標は最大到達円の内側なので幾何的には到達可能です。",
    },
    "extended-out": {
      id: "extended-out",
      label: "伸ばし切り・外側（到達不能）",
      theta1: 0,
      theta2: 0,
      delta: [5, 0],
      hint: "θ₁=0°, θ₂=0°。目標は L₁+L₂ より外側で、そもそも到達できません。",
    },
  };

  class ExerciseController {
    /**
     * @param {{
     *   armView: import('./arm-view.js'),
     *   angleControls: import('./controls.js'),
     *   panel: HTMLElement,
     *   linkLength1?: number,
     *   linkLength2?: number
     * }} deps
     */
    constructor(deps) {
      this.armView = deps.armView;
      this.angleControls = deps.angleControls;
      this.panel = deps.panel;
      this.L1 = deps.linkLength1 ?? LINK_LENGTH_1;
      this.L2 = deps.linkLength2 ?? LINK_LENGTH_2;
      this.currentId = "predict";
      this.correctionScenario = "bent";
      this._correctionApplied = false;
      this._startPose = null;
      this._target = null;
    }

    start(exerciseId) {
      this.currentId = exerciseId;
      this._correctionApplied = false;
      this._target = null;
      this._startPose = null;
      if (exerciseId === "predict") {
        this.angleControls.setEnabled(true);
        this._renderPredict();
        this.angleControls.setAngles(30, 45, true);
      } else {
        // 演習2はシナリオ固定。スライダーを動かすと開始姿勢と目標がずれるため無効化する。
        this.angleControls.setEnabled(false);
        this._loadCorrectionScenario(this.correctionScenario);
      }
      this._syncArm();
    }

    onAnglesChanged(theta1, theta2) {
      this._syncArm();
      if (this.currentId === "predict") {
        this._updatePredictLive(theta1, theta2);
      }
    }

    _syncArm() {
      const { theta1, theta2 } = this.angleControls.getAngles();
      this.angleControls.updateReadout({
        linkLength1: this.L1,
        linkLength2: this.L2,
        theta1,
        theta2,
      });
      const residualFrom =
        this._correctionApplied && this._target
          ? global.RoboticsKinematics.forwardKinematics2Link(
              this.L1,
              this.L2,
              theta1,
              theta2
            ).wrist_world
          : null;
      this.armView.setState({
        linkLength1: this.L1,
        linkLength2: this.L2,
        theta1Deg: theta1,
        theta2Deg: theta2,
        target: this._target,
        residualFrom,
        showWorkspace: this.currentId === "correct",
      });
    }

    _renderPredict() {
      this.panel.innerHTML = `
        <section class="exercise-block">
          <h2>${EXERCISE_META.predict.title}</h2>
          <p class="lede">${EXERCISE_META.predict.summary}</p>
          <p>リンク長は <strong>L₁ = ${this.L1} mm</strong>、<strong>L₂ = ${this.L2} mm</strong> です。
          手先位置は次式です（θ は度、計算時はラジアンへ変換）。</p>
          <pre class="formula">x = L₁ cos θ₁ + L₂ cos(θ₁+θ₂)
y = L₁ sin θ₁ + L₂ sin(θ₁+θ₂)</pre>

          <div class="quiz" data-role="predict-quiz">
            <h3>操作前の予想</h3>
            <p>初期姿勢を <strong>θ₁ = 30°</strong>、<strong>θ₂ = 45°</strong>（前腕の World 角 = 75°）とします。
            手先のおおよその象限と、x・y の大小関係を選んでください。</p>
            <fieldset>
              <legend>手先はどの象限？</legend>
              <label><input type="radio" name="quadrant" value="1"> 第1象限（x&gt;0, y&gt;0）</label>
              <label><input type="radio" name="quadrant" value="2"> 第2象限（x&lt;0, y&gt;0）</label>
              <label><input type="radio" name="quadrant" value="3"> 第3象限（x&lt;0, y&lt;0）</label>
              <label><input type="radio" name="quadrant" value="4"> 第4象限（x&gt;0, y&lt;0）</label>
            </fieldset>
            <fieldset>
              <legend>x と y の関係（おおよそ）</legend>
              <label><input type="radio" name="xyrel" value="x_gt_y"> x の方が大きい</label>
              <label><input type="radio" name="xyrel" value="y_gt_x"> y の方が大きい</label>
              <label><input type="radio" name="xyrel" value="near"> ほぼ同じくらい</label>
            </fieldset>
            <button type="button" class="btn primary" data-action="lock-predict">予想を確定して操作する</button>
            <p class="status" data-role="predict-status" hidden></p>
          </div>

          <div class="live-box" data-role="predict-live" hidden>
            <h3>操作して確かめる</h3>
            <p>左のスライダーで角度を変え、右上の数値と図の手先位置を見てください。
            代表姿勢ボタンでも切り替えられます。</p>
            <div class="preset-row" data-role="predict-presets"></div>
            <p data-role="predict-live-numbers"></p>
            <details class="explain">
              <summary>操作後に開く短い解説</summary>
              <div>
                <p>θ₁=30°, θ₂=45° のとき、肘は約 (129.9, 75.0) mm、手先は約 (161.0, 190.9) mm です
                （Python の Case C を 30 倍した値と同じ）。</p>
                <p>前腕の向きは <strong>θ₁+θ₂ = 75°</strong> です。θ₂ だけを動かしても肩の向きは変わらず、
                肘を支点に前腕だけが回ります。θ₁ を動かすと肘の World 位置そのものが動きます。</p>
                <p>計算はブラウザの <code>forwardKinematics2Link</code> が行い、図はその結果を描いているだけです。
                Python の <code>forward_kinematics_2link</code> と同じ式です。</p>
              </div>
            </details>
          </div>
        </section>
      `;

      const quiz = this.panel.querySelector('[data-role="predict-quiz"]');
      quiz.querySelector('[data-action="lock-predict"]').addEventListener("click", () => {
        const q = quiz.querySelector('input[name="quadrant"]:checked');
        const r = quiz.querySelector('input[name="xyrel"]:checked');
        const status = quiz.querySelector('[data-role="predict-status"]');
        if (!q || !r) {
          status.hidden = false;
          status.textContent = "象限と x・y の関係の両方を選んでから確定してください。";
          return;
        }
        const pose = global.RoboticsKinematics.forwardKinematics2Link(this.L1, this.L2, 30, 45);
        const [wx, wy] = pose.wrist_world;
        const qOk = q.value === "1";
        const relOk =
          (r.value === "y_gt_x" && wy > wx) ||
          (r.value === "x_gt_y" && wx > wy) ||
          (r.value === "near" && Math.abs(wx - wy) < 20);
        status.hidden = false;
        status.textContent = qOk && relOk
          ? "予想はおおよそ合っています。スライダーで数値を確認しましょう。"
          : `正解に近いのは「第1象限」かつ「y の方が大きい」です（実際は x≈${global.RoboticsKinematics.formatNumber(wx)}, y≈${global.RoboticsKinematics.formatNumber(wy)}）。操作で確かめてください。`;
        const live = this.panel.querySelector('[data-role="predict-live"]');
        live.hidden = false;
        this._buildPredictPresets();
        this.angleControls.setAngles(30, 45, true);
      });
    }

    _buildPredictPresets() {
      const row = this.panel.querySelector('[data-role="predict-presets"]');
      if (!row || row.dataset.ready) return;
      row.dataset.ready = "1";
      const cases = [
        { label: "A (0°, 0°)", t1: 0, t2: 0 },
        { label: "B (30°, 0°)", t1: 30, t2: 0 },
        { label: "C (30°, 45°)", t1: 30, t2: 45 },
        { label: "D (90°, −45°)", t1: 90, t2: -45 },
      ];
      for (const c of cases) {
        const btn = document.createElement("button");
        btn.type = "button";
        btn.className = "btn";
        btn.textContent = c.label;
        btn.addEventListener("click", () => this.angleControls.setAngles(c.t1, c.t2, true));
        row.appendChild(btn);
      }
    }

    _updatePredictLive(theta1, theta2) {
      const el = this.panel.querySelector('[data-role="predict-live-numbers"]');
      if (!el || el.closest("[hidden]")) return;
      const pose = global.RoboticsKinematics.forwardKinematics2Link(
        this.L1,
        this.L2,
        theta1,
        theta2
      );
      const fmt = global.RoboticsKinematics.formatNumber;
      el.textContent = `いまの手先: (${fmt(pose.wrist_world[0])}, ${fmt(pose.wrist_world[1])}) mm ／ 前腕 World 角: ${fmt(pose.forearm_angle_world_deg, 1)}°`;
    }

    _loadCorrectionScenario(scenarioId) {
      this.correctionScenario = scenarioId;
      this._correctionApplied = false;
      const sc = CORRECTION_SCENARIOS[scenarioId];
      this.angleControls.setAngles(sc.theta1, sc.theta2, false);
      const start = global.RoboticsKinematics.forwardKinematics2Link(
        this.L1,
        this.L2,
        sc.theta1,
        sc.theta2
      );
      this._startPose = {
        theta1: sc.theta1,
        theta2: sc.theta2,
        wrist: start.wrist_world,
      };
      this._target = [
        start.wrist_world[0] + sc.delta[0],
        start.wrist_world[1] + sc.delta[1],
      ];
      this._renderCorrect();
      this._syncArm();
    }

    _renderCorrect() {
      const sc = CORRECTION_SCENARIOS[this.correctionScenario];
      const fmt = global.RoboticsKinematics.formatNumber;
      const start = this._startPose.wrist;
      const target = this._target;
      const reachable = global.RoboticsKinematics.isReachable(
        this.L1,
        this.L2,
        target[0],
        target[1]
      );
      const rMax = this.L1 + this.L2;
      const rMin = Math.abs(this.L1 - this.L2);

      this.panel.innerHTML = `
        <section class="exercise-block">
          <h2>${EXERCISE_META.correct.title}</h2>
          <p class="lede">${EXERCISE_META.correct.summary}</p>
          <p>微小な手先変位 Δp と関節角変位 Δθ は、ヤコビ行列で一次近似できます。</p>
          <pre class="formula">Δp ≈ J(θ) Δθ
Δθ ≈ J(θ)⁻¹ Δp　（1回だけ解く）</pre>

          <div class="scenario-tabs" data-role="scenario-tabs" role="group" aria-label="補正シナリオ">
            ${Object.values(CORRECTION_SCENARIOS)
              .map(
                (s) =>
                  `<button type="button" class="btn ${s.id === sc.id ? "primary" : ""}" data-scenario="${s.id}" aria-pressed="${s.id === sc.id ? "true" : "false"}">${s.label}</button>`
              )
              .join("")}
          </div>
          <p class="hint">${sc.hint}</p>

          <div class="quiz" data-role="correct-quiz">
            <h3>操作前の予想</h3>
            <p>開始手先 = (${fmt(start[0])}, ${fmt(start[1])}) mm、
            目標 = (${fmt(target[0])}, ${fmt(target[1])}) mm（Δ = (${sc.delta[0]}, ${sc.delta[1]}) mm）。
            作業領域は ${fmt(rMin, 0)}〜${fmt(rMax, 0)} mm です。</p>
            <fieldset>
              <legend>この目標について、いちばん近い予想は？</legend>
              <label><input type="radio" name="expect" value="good"> 1回の一次補正で残差は小さくなる</label>
              <label><input type="radio" name="expect" value="linear-limit"> 幾何的には到達できそうだが、一次近似ではうまく動けない</label>
              <label><input type="radio" name="expect" value="unreachable"> 作業領域の外なので到達不能</label>
            </fieldset>
            <button type="button" class="btn primary" data-action="lock-correct">予想を確定して補正する</button>
            <p class="status" data-role="correct-status" hidden></p>
          </div>

          <div class="live-box" data-role="correct-live" hidden>
            <h3>1回補正の結果</h3>
            <div data-role="correct-result"></div>
            <button type="button" class="btn" data-action="reset-correct">開始姿勢に戻す</button>
            <details class="explain" data-role="correct-explain">
              <summary>操作後に開く短い解説</summary>
              <div data-role="correct-explain-body"></div>
            </details>
          </div>
        </section>
      `;

      this.panel.querySelectorAll("[data-scenario]").forEach((btn) => {
        btn.addEventListener("click", () => {
          this._loadCorrectionScenario(btn.getAttribute("data-scenario"));
        });
      });

      this.panel.querySelector('[data-action="lock-correct"]').addEventListener("click", () => {
        this._applyCorrectionOnce(reachable);
      });
      this.panel.querySelector('[data-action="reset-correct"]')?.addEventListener("click", () => {
        this._loadCorrectionScenario(this.correctionScenario);
      });
    }

    _applyCorrectionOnce(reachableExpected) {
      const quiz = this.panel.querySelector('[data-role="correct-quiz"]');
      const choice = quiz.querySelector('input[name="expect"]:checked');
      const status = quiz.querySelector('[data-role="correct-status"]');
      if (!choice) {
        status.hidden = false;
        status.textContent = "予想を1つ選んでから確定してください。";
        return;
      }

      const sc = CORRECTION_SCENARIOS[this.correctionScenario];
      const startT1 = this._startPose.theta1;
      const startT2 = this._startPose.theta2;
      const dx = sc.delta[0];
      const dy = sc.delta[1];

      const sol = global.RoboticsKinematics.solveJointCorrection(
        this.L1,
        this.L2,
        startT1,
        startT2,
        dx,
        dy
      );
      const newT1 = startT1 + sol.dTheta1Deg;
      const newT2 = startT2 + sol.dTheta2Deg;
      const after = global.RoboticsKinematics.forwardKinematics2Link(
        this.L1,
        this.L2,
        newT1,
        newT2
      );
      const residual = Math.hypot(
        after.wrist_world[0] - this._target[0],
        after.wrist_world[1] - this._target[1]
      );
      const reachable = global.RoboticsKinematics.isReachable(
        this.L1,
        this.L2,
        this._target[0],
        this._target[1]
      );

      let expectedKey;
      if (!reachable) expectedKey = "unreachable";
      else if (sol.singular || residual > 1.0) expectedKey = "linear-limit";
      else expectedKey = "good";

      status.hidden = false;
      status.textContent =
        choice.value === expectedKey
          ? "予想は的確です。結果の数値を確認してください。"
          : `このシナリオで特に見てほしいのは「${labelForExpect(expectedKey)}」です。結果を見比べましょう。`;

      this._correctionApplied = true;
      this.angleControls.setAngles(newT1, newT2, true);

      const fmt = global.RoboticsKinematics.formatNumber;
      const live = this.panel.querySelector('[data-role="correct-live"]');
      live.hidden = false;
      live.querySelector('[data-role="correct-result"]').innerHTML = `
        <ul class="result-list">
          <li>Δθ₁ = ${fmt(sol.dTheta1Deg, 3)}°, Δθ₂ = ${fmt(sol.dTheta2Deg, 3)}°</li>
          <li>補正後の関節角: θ₁ = ${fmt(newT1, 3)}°, θ₂ = ${fmt(newT2, 3)}°</li>
          <li>補正後の手先: (${fmt(after.wrist_world[0])}, ${fmt(after.wrist_world[1])}) mm</li>
          <li>目標までの残差: <strong>${fmt(residual, 3)} mm</strong></li>
          <li>det(J) = ${fmt(sol.det, 2)} ${sol.singular ? "（特異に近い）" : ""}</li>
          <li>目標の到達可能性: ${reachable ? "到達可能（作業領域内）" : "到達不能（作業領域外）"}</li>
          <li>一次近似が予測した手先変位: (${fmt(sol.predictedDelta[0], 3)}, ${fmt(sol.predictedDelta[1], 3)}) mm</li>
        </ul>
      `;

      live.querySelector('[data-role="correct-explain-body"]').innerHTML =
        explainCorrection(sc.id, {
          residual,
          singular: sol.singular,
          reachable,
          rMax: this.L1 + this.L2,
        });

      // re-bind reset (innerHTML replaced scenario panel earlier; buttons exist)
      live.querySelector('[data-action="reset-correct"]').onclick = () => {
        this._loadCorrectionScenario(this.correctionScenario);
      };
    }
  }

  function labelForExpect(key) {
    if (key === "good") return "1回の一次補正で残差は小さくなる";
    if (key === "linear-limit") return "到達可能だが一次近似の限界";
    return "到達不能";
  }

  function explainCorrection(scenarioId, info) {
    if (scenarioId === "bent") {
      return `
        <p>肘を曲げた姿勢ではヤコビ行列が正則で、5 mm 程度の変位なら1回の一次補正で残差は
        だいたい ${global.RoboticsKinematics.formatNumber(info.residual, 2)} mm まで落ちます
        （完全にゼロにはなりません。これが線形近似の誤差です）。</p>
        <p>同じ 5 mm でも、伸ばし切った姿勢では様子が変わります。上のタブで比較してください。</p>
      `;
    }
    if (scenarioId === "extended-in") {
      return `
        <p>目標半径 ${global.RoboticsKinematics.formatNumber(info.rMax - 5, 0)} mm は最大到達
        ${global.RoboticsKinematics.formatNumber(info.rMax, 0)} mm より内側なので<strong>到達可能</strong>です。
        実際には肘を曲げれば手先を内側へ落とせます。</p>
        <p>しかし伸ばし切り（θ₂=0）は特異姿勢で、ヤコビ行列の列が一次従属になり、
        半径方向の動きを線形では作れません。その結果 Δθ≈0 のまま残差が約 5 mm 残ります。
        これが<strong>一次近似の限界</strong>です（到達不能とは別の失敗モード）。</p>
      `;
    }
    return `
      <p>目標半径 ${global.RoboticsKinematics.formatNumber(info.rMax + 5, 0)} mm は最大到達
      ${global.RoboticsKinematics.formatNumber(info.rMax, 0)} mm を超えているので<strong>到達不能</strong>です。
      何度ニュートン法を回しても幾何的に届きません。</p>
      <p>特異姿勢でもあるため一次補正は動きませんが、本質的な理由は作業領域の外にあることです。
      内側シナリオの「一次近似の限界」と対比して覚えてください。</p>
    `;
  }

  global.ExerciseMeta = EXERCISE_META;
  global.CorrectionScenarios = CORRECTION_SCENARIOS;
  global.ExerciseController = ExerciseController;
  global.LabConstants = { LINK_LENGTH_1, LINK_LENGTH_2 };
})(typeof window !== "undefined" ? window : globalThis);
