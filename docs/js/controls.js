/**
 * 角度スライダーと数値表示の共通コントロール。
 */
(function (global) {
  "use strict";

  /**
   * @typedef {{
   *   theta1: number,
   *   theta2: number,
   *   onChange: (theta1: number, theta2: number, source: string) => void
   * }} AngleControlsOptions
   */

  class AngleControls {
    /**
     * @param {HTMLElement} root
     * @param {AngleControlsOptions} options
     */
    constructor(root, options) {
      this.root = root;
      this.onChange = options.onChange;
      this._locked = false;

      this.slider1 = root.querySelector('[data-role="theta1-slider"]');
      this.slider2 = root.querySelector('[data-role="theta2-slider"]');
      this.input1 = root.querySelector('[data-role="theta1-input"]');
      this.input2 = root.querySelector('[data-role="theta2-input"]');
      this.readout = root.querySelector('[data-role="pose-readout"]');

      const emit = (source) => {
        if (this._locked) return;
        const t1 = Number(this.slider1.value);
        const t2 = Number(this.slider2.value);
        this.input1.value = String(t1);
        this.input2.value = String(t2);
        this.onChange(t1, t2, source);
      };

      this.slider1.addEventListener("input", () => emit("slider"));
      this.slider2.addEventListener("input", () => emit("slider"));

      const syncFromInput = (which) => {
        const el = which === 1 ? this.input1 : this.input2;
        const slider = which === 1 ? this.slider1 : this.slider2;
        let v = Number(el.value);
        if (!Number.isFinite(v)) return;
        v = Math.max(Number(slider.min), Math.min(Number(slider.max), v));
        slider.value = String(v);
        el.value = String(v);
        emit("input");
      };
      this.input1.addEventListener("change", () => syncFromInput(1));
      this.input2.addEventListener("change", () => syncFromInput(2));

      this.setAngles(options.theta1, options.theta2, false);
    }

    setAngles(theta1, theta2, notify = true) {
      this._locked = true;
      this.slider1.value = String(theta1);
      this.slider2.value = String(theta2);
      this.input1.value = String(theta1);
      this.input2.value = String(theta2);
      this._locked = false;
      if (notify) this.onChange(theta1, theta2, "program");
    }

    getAngles() {
      return {
        theta1: Number(this.slider1.value),
        theta2: Number(this.slider2.value),
      };
    }

    setEnabled(enabled) {
      this.slider1.disabled = !enabled;
      this.slider2.disabled = !enabled;
      this.input1.disabled = !enabled;
      this.input2.disabled = !enabled;
    }

    /**
     * @param {{
     *   linkLength1: number,
     *   linkLength2: number,
     *   theta1: number,
     *   theta2: number
     * }} poseInput
     */
    updateReadout(poseInput) {
      if (!this.readout) return;
      const pose = global.RoboticsKinematics.forwardKinematics2Link(
        poseInput.linkLength1,
        poseInput.linkLength2,
        poseInput.theta1,
        poseInput.theta2
      );
      const fmt = global.RoboticsKinematics.formatNumber;
      const [wx, wy] = pose.wrist_world;
      const [ex, ey] = pose.elbow_world;
      this.readout.innerHTML = [
        `<div><span>θ₁</span> ${fmt(poseInput.theta1, 1)}°</div>`,
        `<div><span>θ₂</span> ${fmt(poseInput.theta2, 1)}°</div>`,
        `<div><span>肘</span> (${fmt(ex)}, ${fmt(ey)}) mm</div>`,
        `<div><span>手先</span> (${fmt(wx)}, ${fmt(wy)}) mm</div>`,
        `<div><span>到達半径</span> ${fmt(Math.hypot(wx, wy))} mm</div>`,
      ].join("");
    }
  }

  global.AngleControls = AngleControls;
})(typeof window !== "undefined" ? window : globalThis);
