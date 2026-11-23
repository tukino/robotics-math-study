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
      title: "演習1：手先はどこに来る？",
      summary: "角度を見て手先位置を予想し、図で確かめる。",
    },
    correct: {
      id: "correct",
      title: "演習2：5 mm 動かしたいとき",
      summary: "関節角を1回だけ補正し、残差と到達不能を区別する。",
    },
  };

  /**
   * 演習2のシナリオ（推奨順）。
   */
  const CORRECTION_SCENARIOS = {
    bent: {
      id: "bent",
      order: 1,
      label: "比較1：肘を曲げた姿勢",
      theta1: 30,
      theta2: 45,
      delta: [5, 0],
      look: "緑の目標点が、いまの手先から少し右（+x）にあること。",
      think: "この姿勢なら、1回の一次補正で残差はかなり小さくなるはずか？",
      hint: "θ₁=30°, θ₂=45°。ヤコビ行列は正則です。",
    },
    "extended-in": {
      id: "extended-in",
      order: 2,
      label: "比較2：伸ばし切り・内側",
      theta1: 0,
      theta2: 0,
      delta: [-5, 0],
      look: "目標が最大到達円の内側にあること（緑の円の内側）。",
      think: "到達はできそうでも、伸ばし切りでは一次近似が効かないのでは？",
      hint: "θ₁=0°, θ₂=0°（特異姿勢）。幾何的には到達可能です。",
    },
    "extended-out": {
      id: "extended-out",
      order: 3,
      label: "比較3：伸ばし切り・外側",
      theta1: 0,
      theta2: 0,
      delta: [5, 0],
      look: "目標が最大到達円の外側にあること。",
      think: "これは一次近似の問題ではなく、そもそも届かないのでは？",
      hint: "目標は L₁+L₂ より外側で、到達不能です。",
    },
  };

  class ExerciseController {
    /**
     * @param {{
     *   armView: *,
     *   angleControls: *,
     *   panel: HTMLElement,
     *   onProgress?: (info: {step: string, message: string, stageHint?: string, controlsHint?: string}) => void,
     *   onRequestExercise?: (id: string) => void,
     *   linkLength1?: number,
     *   linkLength2?: number
     * }} deps
     */
    constructor(deps) {
      this.armView = deps.armView;
      this.angleControls = deps.angleControls;
      this.panel = deps.panel;
      this.onProgress = deps.onProgress || (() => {});
      this.onRequestExercise = deps.onRequestExercise || (() => {});
      this.L1 = deps.linkLength1 ?? LINK_LENGTH_1;
      this.L2 = deps.linkLength2 ?? LINK_LENGTH_2;
      this.currentId = "predict";
      this.correctionScenario = "bent";
      this._correctionApplied = false;
      this._startPose = null;
      this._target = null;
      this._completedScenarios = new Set();
    }

    start(exerciseId) {
      this.currentId = exerciseId;
      this._correctionApplied = false;
      this._target = null;
      this._startPose = null;
      if (exerciseId === "predict") {
        this.angleControls.setEnabled(false);
        this._renderPredict();
        this.angleControls.setAngles(30, 45, true);
        this._setProgress(
          "look",
          "いまやること：右の手順で、図のどこを見るかを確認する。",
          "青い上腕・オレンジ前腕・赤い手先を確認してください。",
          "まだ動かさないでください。先に手順を読みます。"
        );
      } else {
        this.angleControls.setEnabled(false);
        this._loadCorrectionScenario(this.correctionScenario || "bent");
      }
      this._syncArm();
    }

    onAnglesChanged(theta1, theta2) {
      this._syncArm();
      if (this.currentId === "predict") {
        this._updatePredictLive(theta1, theta2);
      }
    }

    _setProgress(step, message, stageHint, controlsHint) {
      this.onProgress({ step, message, stageHint, controlsHint });
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

    _cueBox(look, think, next) {
      return `
        <div class="cue-box" aria-label="いまの見方と考え方">
          <p><span class="cue-label">見てほしいもの</span>${look}</p>
          <p><span class="cue-label">考えてほしいこと</span>${think}</p>
          <p><span class="cue-label">次にやること</span>${next}</p>
        </div>
      `;
    }

    _renderPredict() {
      this.panel.innerHTML = `
        <section class="exercise-block">
          <p class="exercise-kicker">演習 1 / 2</p>
          <h2>${EXERCISE_META.predict.title}</h2>
          <p class="lede">${EXERCISE_META.predict.summary}</p>

          <div class="step-card is-open" data-role="step-look">
            <h3><span class="step-num">1</span> まず図を見る</h3>
            ${this._cueBox(
              "右の実験台で、肩（黒）・肘（紫）・手先（赤）の位置。",
              "θ₁=30°, θ₂=45° のとき、手先はだいたいどの象限にありそうか。",
              "下の「予想へ進む」を押す。"
            )}
            <p>リンク長は L₁=${this.L1} mm、L₂=${this.L2} mm。初期角は θ₁=30°、θ₂=45° です。</p>
            <details class="formula-details">
              <summary>式を見る（任意）</summary>
              <pre class="formula">x = L₁ cos θ₁ + L₂ cos(θ₁+θ₂)
y = L₁ sin θ₁ + L₂ sin(θ₁+θ₂)</pre>
              <p class="hint">前腕の World 角は θ₁+θ₂（ここでは 75°）です。</p>
            </details>
            <button type="button" class="btn primary" data-action="to-predict">予想へ進む</button>
          </div>

          <div class="step-card" data-role="step-predict" hidden>
            <h3><span class="step-num">2</span> 操作前に予想する</h3>
            ${this._cueBox(
              "まだスライダーは動かさない。数値の象限だけを選ぶ。",
              "手先は第何象限か。x と y のどちらが大きそうか。",
              "選択肢を選んで「予想を確定する」。"
            )}
            <div class="quiz" data-role="predict-quiz">
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
              <button type="button" class="btn primary" data-action="lock-predict">予想を確定する</button>
              <p class="status" data-role="predict-status" hidden></p>
            </div>
          </div>

          <div class="step-card" data-role="step-check" hidden>
            <h3><span class="step-num">3</span> 操作して確かめる</h3>
            ${this._cueBox(
              "右の実験台の手先座標と、図上の赤い点。",
              "予想と数値は一致したか。θ₁ と θ₂ を動かすと何が変わるか。",
              "スライダーか代表姿勢を試し、終わったら演習2へ。"
            )}
            <div class="live-box" data-role="predict-live">
              <p>下のスライダーが使えるようになりました。代表姿勢でも切り替えられます。</p>
              <div class="preset-row" data-role="predict-presets"></div>
              <p data-role="predict-live-numbers"></p>
              <details class="explain">
                <summary>短い解説を開く</summary>
                <div>
                  <p>θ₁=30°, θ₂=45° のとき、肘は約 (129.9, 75.0) mm、手先は約 (161.0, 190.9) mm です
                  （Python の Case C を 30 倍した値と同じ）。</p>
                  <p>前腕の向きは <strong>θ₁+θ₂ = 75°</strong> です。θ₂ だけなら肘まわり、θ₁ なら肘の位置ごと動きます。</p>
                </div>
              </details>
            </div>
            <button type="button" class="btn primary" data-action="to-exercise-2">次へ：演習2（5 mm 補正）</button>
          </div>
        </section>
      `;

      this.panel.querySelector('[data-action="to-predict"]').addEventListener("click", () => {
        this.panel.querySelector('[data-role="step-look"]').classList.remove("is-open");
        const next = this.panel.querySelector('[data-role="step-predict"]');
        next.hidden = false;
        next.classList.add("is-open");
        next.scrollIntoView({ behavior: "smooth", block: "nearest" });
        this._setProgress(
          "predict",
          "いまやること：象限と x・y の大小を選んで予想を確定する。",
          "図は参考に見てよいですが、まだスライダーは触らないでください。",
          "まだロック中です。予想を先に出します。"
        );
      });

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
          ? "予想はおおよそ合っています。次で数値を確かめましょう。"
          : `おおよそは「第1象限」かつ「y の方が大きい」です（実際は x≈${global.RoboticsKinematics.formatNumber(wx)}, y≈${global.RoboticsKinematics.formatNumber(wy)}）。次で図を動かして確認してください。`;

        this.panel.querySelector('[data-role="step-predict"]').classList.remove("is-open");
        const check = this.panel.querySelector('[data-role="step-check"]');
        check.hidden = false;
        check.classList.add("is-open");
        this._buildPredictPresets();
        this.angleControls.setEnabled(true);
        this.angleControls.setAngles(30, 45, true);
        check.scrollIntoView({ behavior: "smooth", block: "nearest" });
        this._setProgress(
          "check",
          "いまやること：スライダーか代表姿勢で数値を確かめ、終わったら演習2へ。",
          "手先の数字と赤い点が連動することを確認してください。",
          "操作して大丈夫です。"
        );
      });

      this.panel.querySelector('[data-action="to-exercise-2"]').addEventListener("click", () => {
        this.onRequestExercise("correct");
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
      if (!el || this.panel.querySelector('[data-role="step-check"][hidden]')) return;
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
      this.angleControls.setEnabled(false);
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
      const progressStep = scenarioId === "bent" ? "correct" : "compare";
      this._setProgress(
        progressStep,
        `いまやること：${sc.label}で予想を選び、「補正する」を押す。`,
        sc.look + " スライダーはシナリオ固定のためロックされています。",
        "演習2では角度は固定です。シナリオ切替だけで進めます。"
      );
    }

    _renderCorrect() {
      const sc = CORRECTION_SCENARIOS[this.correctionScenario];
      const fmt = global.RoboticsKinematics.formatNumber;
      const start = this._startPose.wrist;
      const target = this._target;
      const rMax = this.L1 + this.L2;
      const rMin = Math.abs(this.L1 - this.L2);
      const scenarios = Object.values(CORRECTION_SCENARIOS).sort((a, b) => a.order - b.order);

      this.panel.innerHTML = `
        <section class="exercise-block">
          <p class="exercise-kicker">演習 2 / 2</p>
          <h2>${EXERCISE_META.correct.title}</h2>
          <p class="lede">${EXERCISE_META.correct.summary}</p>
          <p class="path-note">推奨順は比較1 → 比較2 → 比較3です。同じ 5 mm でも結果が違います。</p>

          ${this._cueBox(sc.look, sc.think, "下で予想を選び、「予想を確定して補正する」を押す。")}

          <div class="scenario-tabs" data-role="scenario-tabs" role="group" aria-label="補正シナリオ">
            ${scenarios
              .map(
                (s) =>
                  `<button type="button" class="btn ${s.id === sc.id ? "primary" : ""}" data-scenario="${s.id}" aria-pressed="${s.id === sc.id ? "true" : "false"}">${s.label}${this._completedScenarios.has(s.id) ? " ✓" : ""}</button>`
              )
              .join("")}
          </div>
          <p class="hint">${sc.hint}</p>

          <details class="formula-details">
            <summary>使っている近似（任意）</summary>
            <pre class="formula">Δp ≈ J(θ) Δθ
Δθ ≈ J(θ)⁻¹ Δp　（1回だけ解く）</pre>
          </details>

          <div class="quiz" data-role="correct-quiz">
            <h3>操作前の予想</h3>
            <p>開始手先 = (${fmt(start[0])}, ${fmt(start[1])}) mm、
            目標 = (${fmt(target[0])}, ${fmt(target[1])}) mm（Δ = (${sc.delta[0]}, ${sc.delta[1]}) mm）。
            作業領域は ${fmt(rMin, 0)}〜${fmt(rMax, 0)} mm です。</p>
            <fieldset>
              <legend>いちばん近い予想は？</legend>
              <label><input type="radio" name="expect" value="good"> 1回の一次補正で残差は小さくなる</label>
              <label><input type="radio" name="expect" value="linear-limit"> 到達できそうだが、一次近似ではうまく動けない</label>
              <label><input type="radio" name="expect" value="unreachable"> 作業領域の外なので到達不能</label>
            </fieldset>
            <button type="button" class="btn primary" data-action="lock-correct">予想を確定して補正する</button>
            <p class="status" data-role="correct-status" hidden></p>
          </div>

          <div class="live-box" data-role="correct-live" hidden>
            <h3>1回補正の結果</h3>
            <div data-role="correct-result"></div>
            <details class="explain" open data-role="correct-explain">
              <summary>短い解説</summary>
              <div data-role="correct-explain-body"></div>
            </details>
            <div class="next-actions" data-role="correct-next"></div>
          </div>
        </section>
      `;

      this.panel.querySelectorAll("[data-scenario]").forEach((btn) => {
        btn.addEventListener("click", () => {
          this._loadCorrectionScenario(btn.getAttribute("data-scenario"));
        });
      });

      this.panel.querySelector('[data-action="lock-correct"]').addEventListener("click", () => {
        this._applyCorrectionOnce();
      });
    }

    _applyCorrectionOnce() {
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
          ? "予想は的確です。下の結果と解説を読んでください。"
          : `この比較で見てほしいのは「${labelForExpect(expectedKey)}」です。結果を確認しましょう。`;

      this._correctionApplied = true;
      this._completedScenarios.add(sc.id);
      this.angleControls.setAngles(newT1, newT2, true);

      // 補正後は「次にやること」を更新し、質問欄は結果の下で参照用に残す
      const cueNext = this.panel.querySelector(".cue-box p:last-child");
      const nextId = nextScenarioId(sc.id);
      if (cueNext) {
        cueNext.innerHTML = nextId
          ? `<span class="cue-label">次にやること</span>解説を読んだら「${CORRECTION_SCENARIOS[nextId].label}」へ進む。`
          : `<span class="cue-label">次にやること</span>3比較の違いを振り返る。必要ならやり直す。`;
      }
      this.panel.querySelectorAll("[data-scenario]").forEach((btn) => {
        const id = btn.getAttribute("data-scenario");
        const meta = CORRECTION_SCENARIOS[id];
        btn.textContent = `${meta.label}${this._completedScenarios.has(id) ? " ✓" : ""}`;
      });

      const fmt = global.RoboticsKinematics.formatNumber;
      const live = this.panel.querySelector('[data-role="correct-live"]');
      live.hidden = false;
      live.querySelector('[data-role="correct-result"]').innerHTML = `
        <ul class="result-list">
          <li>残差: <strong>${fmt(residual, 3)} mm</strong></li>
          <li>到達可能性: ${reachable ? "到達可能" : "到達不能"}</li>
          <li>Δθ₁ = ${fmt(sol.dTheta1Deg, 3)}°, Δθ₂ = ${fmt(sol.dTheta2Deg, 3)}°</li>
          <li>det(J) = ${fmt(sol.det, 2)} ${sol.singular ? "（特異に近い）" : ""}</li>
        </ul>
      `;

      live.querySelector('[data-role="correct-explain-body"]').innerHTML =
        explainCorrection(sc.id, {
          residual,
          singular: sol.singular,
          reachable,
          rMax: this.L1 + this.L2,
        });

      const nextBox = live.querySelector('[data-role="correct-next"]');
      if (nextId) {
        const next = CORRECTION_SCENARIOS[nextId];
        nextBox.innerHTML = `
          <button type="button" class="btn primary" data-action="next-scenario">次へ：${next.label}</button>
          <button type="button" class="btn" data-action="retry-scenario">この比較をやり直す</button>
        `;
        nextBox.querySelector('[data-action="next-scenario"]').onclick = () => {
          this._loadCorrectionScenario(nextId);
        };
        this._setProgress(
          "compare",
          `いまやること：解説を読んだら「${next.label}」へ進む。`,
          "補正後の手先と目標の距離（残差）に注目してください。",
          "角度は固定のままです。"
        );
      } else {
        nextBox.innerHTML = `
          <p class="done-msg">3つの比較が終わりました。残差が小さい／一次近似の限界／到達不能、の違いを振り返ってください。</p>
          <button type="button" class="btn" data-action="retry-scenario">この比較をやり直す</button>
          <button type="button" class="btn" data-action="back-ex1">演習1に戻る</button>
        `;
        nextBox.querySelector('[data-action="back-ex1"]').onclick = () => {
          this.onRequestExercise("predict");
        };
        this._setProgress(
          "compare",
          "いまやること：3比較の違いを振り返る。必要なら比較をやり直す。",
          "緑の目標と赤い手先のずれ（残差）を見比べてください。",
          "角度は固定のままです。"
        );
      }
      nextBox.querySelector('[data-action="retry-scenario"]').onclick = () => {
        this._loadCorrectionScenario(this.correctionScenario);
      };

      live.scrollIntoView({ behavior: "smooth", block: "nearest" });
    }
  }

  function nextScenarioId(currentId) {
    if (currentId === "bent") return "extended-in";
    if (currentId === "extended-in") return "extended-out";
    return null;
  }

  function labelForExpect(key) {
    if (key === "good") return "1回の一次補正で残差は小さくなる";
    if (key === "linear-limit") return "到達可能だが一次近似の限界";
    return "到達不能";
  }

  function explainCorrection(scenarioId, info) {
    if (scenarioId === "bent") {
      return `
        <p>肘を曲げた姿勢ではヤコビ行列が正則で、5 mm 程度なら1回の一次補正で残差は
        だいたい ${global.RoboticsKinematics.formatNumber(info.residual, 2)} mm まで落ちます。
        ゼロにならない分が線形近似の誤差です。</p>
        <p>次は、同じ 5 mm を伸ばし切り姿勢で試します。</p>
      `;
    }
    if (scenarioId === "extended-in") {
      return `
        <p>目標は最大到達の内側なので<strong>到達可能</strong>です。でも伸ばし切りは特異姿勢なので、
        一次近似では半径方向に動けず残差が約 5 mm 残ります。これが<strong>一次近似の限界</strong>です。</p>
      `;
    }
    return `
      <p>目標は最大到達の外側なので<strong>到達不能</strong>です。近似の精度以前に、幾何的に届きません。
      比較2の「一次近似の限界」と区別して覚えてください。</p>
    `;
  }

  global.ExerciseMeta = EXERCISE_META;
  global.CorrectionScenarios = CORRECTION_SCENARIOS;
  global.ExerciseController = ExerciseController;
  global.LabConstants = { LINK_LENGTH_1, LINK_LENGTH_2 };
})(typeof window !== "undefined" ? window : globalThis);
