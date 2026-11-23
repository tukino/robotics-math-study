/**
 * 2リンクアームの Canvas 描画。演習から再利用する共通ビュー。
 */
(function (global) {
  "use strict";

  /**
   * @typedef {{
   *   linkLength1: number,
   *   linkLength2: number,
   *   theta1Deg: number,
   *   theta2Deg: number,
   *   target?: [number, number] | null,
   *   residualFrom?: [number, number] | null,
   *   showWorkspace?: boolean,
   *   highlightWrist?: boolean
   * }} ArmViewState
   */

  class ArmView {
    /**
     * @param {HTMLCanvasElement} canvas
     * @param {{ padding?: number }} [options]
     */
    constructor(canvas, options = {}) {
      this.canvas = canvas;
      this.ctx = canvas.getContext("2d");
      this.padding = options.padding ?? 28;
      this._resizeObserver = null;
      this.state = {
        linkLength1: 150,
        linkLength2: 120,
        theta1Deg: 30,
        theta2Deg: 45,
        target: null,
        residualFrom: null,
        showWorkspace: true,
        highlightWrist: true,
      };
      this._fitToDevicePixelRatio();
    }

    /**
     * @param {Partial<ArmViewState>} partial
     */
    setState(partial) {
      Object.assign(this.state, partial);
      this.draw();
    }

    attachResize() {
      const redraw = () => {
        this._fitToDevicePixelRatio();
        this.draw();
      };
      window.addEventListener("resize", redraw);
      if (typeof ResizeObserver !== "undefined") {
        this._resizeObserver = new ResizeObserver(redraw);
        this._resizeObserver.observe(this.canvas.parentElement || this.canvas);
      }
    }

    _fitToDevicePixelRatio() {
      const dpr = window.devicePixelRatio || 1;
      const rect = this.canvas.getBoundingClientRect();
      const width = Math.max(320, Math.floor(rect.width));
      const height = Math.max(280, Math.floor(rect.height));
      this.canvas.width = Math.floor(width * dpr);
      this.canvas.height = Math.floor(height * dpr);
      this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      this.cssWidth = width;
      this.cssHeight = height;
    }

    /** World mm → Canvas px */
    _worldToCanvas(x, y) {
      const maxReach = this.state.linkLength1 + this.state.linkLength2;
      const span = maxReach * 1.25;
      const usableW = this.cssWidth - this.padding * 2;
      const usableH = this.cssHeight - this.padding * 2;
      const scale = Math.min(usableW, usableH) / (2 * span);
      const cx = this.cssWidth / 2;
      const cy = this.cssHeight / 2;
      return {
        x: cx + x * scale,
        y: cy - y * scale,
        scale,
      };
    }

    draw() {
      const { ctx, state } = this;
      const w = this.cssWidth;
      const h = this.cssHeight;
      ctx.clearRect(0, 0, w, h);

      // 紙面背景
      ctx.fillStyle = "#f3f0e8";
      ctx.fillRect(0, 0, w, h);
      this._drawGrid();

      const pose = global.RoboticsKinematics.forwardKinematics2Link(
        state.linkLength1,
        state.linkLength2,
        state.theta1Deg,
        state.theta2Deg
      );

      if (state.showWorkspace) {
        this._drawWorkspaceRings(state.linkLength1, state.linkLength2);
      }

      this._drawAxes();

      const shoulder = this._worldToCanvas(...pose.shoulder_world);
      const elbow = this._worldToCanvas(...pose.elbow_world);
      const wrist = this._worldToCanvas(...pose.wrist_world);

      // リンク
      ctx.lineCap = "round";
      ctx.lineWidth = 10;
      ctx.strokeStyle = "#1f4b7a";
      ctx.beginPath();
      ctx.moveTo(shoulder.x, shoulder.y);
      ctx.lineTo(elbow.x, elbow.y);
      ctx.stroke();

      ctx.strokeStyle = "#c45c26";
      ctx.beginPath();
      ctx.moveTo(elbow.x, elbow.y);
      ctx.lineTo(wrist.x, wrist.y);
      ctx.stroke();

      if (state.target) {
        const t = this._worldToCanvas(state.target[0], state.target[1]);
        ctx.strokeStyle = "#0a7a4b";
        ctx.lineWidth = 2;
        ctx.setLineDash([5, 4]);
        ctx.beginPath();
        ctx.moveTo(wrist.x, wrist.y);
        ctx.lineTo(t.x, t.y);
        ctx.stroke();
        ctx.setLineDash([]);

        ctx.fillStyle = "#0a7a4b";
        ctx.beginPath();
        ctx.arc(t.x, t.y, 7, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = "#0a7a4b";
        ctx.font = "12px 'IBM Plex Sans JP', 'Noto Sans JP', sans-serif";
        ctx.fillText("目標", t.x + 10, t.y - 8);
      }

      if (state.residualFrom && state.target) {
        const from = this._worldToCanvas(state.residualFrom[0], state.residualFrom[1]);
        const to = this._worldToCanvas(state.target[0], state.target[1]);
        ctx.strokeStyle = "#a11a2e";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(from.x, from.y);
        ctx.lineTo(to.x, to.y);
        ctx.stroke();
      }

      this._drawJoint(shoulder.x, shoulder.y, "#1a1a1a", 7);
      this._drawJoint(elbow.x, elbow.y, "#6b3fa0", 7);
      this._drawJoint(wrist.x, wrist.y, state.highlightWrist ? "#c1121f" : "#333", 8);

      ctx.fillStyle = "#333";
      ctx.font = "12px 'IBM Plex Sans JP', 'Noto Sans JP', sans-serif";
      ctx.fillText("肩", shoulder.x + 10, shoulder.y + 16);
      ctx.fillText("肘", elbow.x + 10, elbow.y - 10);
      ctx.fillText("手先", wrist.x + 10, wrist.y + 4);
    }

    _drawGrid() {
      const ctx = this.ctx;
      const stepMm = 50;
      const maxReach = this.state.linkLength1 + this.state.linkLength2;
      const span = maxReach * 1.25;
      ctx.strokeStyle = "rgba(40, 55, 70, 0.08)";
      ctx.lineWidth = 1;
      for (let v = -span; v <= span + 0.01; v += stepMm) {
        const a = this._worldToCanvas(v, -span);
        const b = this._worldToCanvas(v, span);
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
        const c = this._worldToCanvas(-span, v);
        const d = this._worldToCanvas(span, v);
        ctx.beginPath();
        ctx.moveTo(c.x, c.y);
        ctx.lineTo(d.x, d.y);
        ctx.stroke();
      }
    }

    _drawAxes() {
      const ctx = this.ctx;
      const origin = this._worldToCanvas(0, 0);
      const tipX = this._worldToCanvas(40, 0);
      const tipY = this._worldToCanvas(0, 40);
      ctx.strokeStyle = "#555";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(origin.x - 30, origin.y);
      ctx.lineTo(tipX.x, tipX.y);
      ctx.moveTo(origin.x, origin.y + 30);
      ctx.lineTo(tipY.x, tipY.y);
      ctx.stroke();
      ctx.fillStyle = "#555";
      ctx.font = "11px 'IBM Plex Sans JP', 'Noto Sans JP', sans-serif";
      ctx.fillText("X", tipX.x + 4, tipX.y + 4);
      ctx.fillText("Y", tipY.x + 4, tipY.y - 4);
    }

    _drawWorkspaceRings(l1, l2) {
      const ctx = this.ctx;
      const origin = this._worldToCanvas(0, 0);
      const rMax = this._worldToCanvas(l1 + l2, 0).x - origin.x;
      const rMin = this._worldToCanvas(Math.abs(l1 - l2), 0).x - origin.x;
      ctx.strokeStyle = "rgba(10, 122, 75, 0.25)";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(origin.x, origin.y, Math.abs(rMax), 0, Math.PI * 2);
      ctx.stroke();
      ctx.strokeStyle = "rgba(10, 122, 75, 0.18)";
      ctx.beginPath();
      ctx.arc(origin.x, origin.y, Math.abs(rMin), 0, Math.PI * 2);
      ctx.stroke();
    }

    _drawJoint(x, y, color, radius) {
      const ctx = this.ctx;
      ctx.fillStyle = color;
      ctx.beginPath();
      ctx.arc(x, y, radius, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = "#fff";
      ctx.beginPath();
      ctx.arc(x, y, radius * 0.35, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  global.ArmView = ArmView;
})(typeof window !== "undefined" ? window : globalThis);
