
export class CanvasRenderer {
  /**
   * @param {HTMLCanvasElement} canvas
   * @param {Object} [options]
   * @param {boolean} [options.showVelocity=false]  Draw velocity vectors
   * @param {number}  [options.velocityScale=0.02]  Pixels per (px/s) for velocity arrow length
   * @param {boolean} [options.showBounds=true]     Draw world bounds if provided
   */

  constructor(canvas, {
    showVelocity = false,
    velocityScale = 0.02,
    showBounds = true
  } = {}) {
    this._canvas = canvas;
    this._ctx = canvas.getContext('2d');
    this._showVelocity = showVelocity;
    this._velocityScale = velocityScale;
    this._showBounds = showBounds;
  }

  SetDebugOptions({ showVelocity, velocityScale, showBounds } = {}) {
    if (typeof showVelocity === 'boolean') this._showVelocity = showVelocity;
    if (typeof velocityScale === 'number') this._velocityScale = velocityScale;
    if (typeof showBounds === 'boolean') this._showBounds = showBounds;
    return this;
  }

  Clear() {
    const c = this._ctx;
    const { width, height } = this._canvas;
    c.clearRect(0, 0, width, height);
  }

  /**
   * Convenience draw: renders bounds (if enabled) and all bodies from a World.
   * Expects the world to have Bodies() and Bounds().
   */
  DrawWorld(world) {
    if (this._showBounds && typeof world.Bounds === 'function') {
      const bounds = world.Bounds();
      if (bounds) this.DrawBounds(bounds);
    }
    if (typeof world.Bodies === 'function') {
      this.DrawBodies(world.Bodies());
    }
  }

  DrawBounds({ minX, minY, maxX, maxY }) {
    const c = this._ctx;
    c.save();
    c.beginPath();
    c.rect(minX, minY, maxX - minX, maxY - minY);
    c.lineWidth = 2;
    c.strokeStyle = '#999';
    c.setLineDash([6, 4]);
    c.stroke();
    c.restore();
  }

  /**
     * Draw an array of bodies (circles).
     * Bodies are expected to have:
     *  - Position() -> {x,y}
     *  - Radius()   -> number
     *  - Velocity() -> {x,y}
     *  - Restitution() -> number (used only for optional debug color tweak)
 */
  DrawBodies(bodies) {
    const c = this._ctx;
    for (const b of bodies) {
      const p = b.Position();
      const r = b.Radius();

      // fill
      c.beginPath();
      c.arc(p.x, p.y, r, 0, Math.PI * 2);
      c.fillStyle = '#2d6cdf';
      c.fill();

      // outline
      c.lineWidth = 1;
      c.strokeStyle = '#111';
      c.stroke();

      // optional: velocity vector
      if (this._showVelocity) this._DrawVelocityArrow(b);
    }
  }

  _DrawVelocityArrow(body) {
    const c = this._ctx;
    const p = body.Position();
    const v = body.Velocity();

    const sx = p.x;
    const sy = p.y;
    const ex = p.x + v.x * this._velocityScale;
    const ey = p.y + v.y * this._velocityScale;

    // line
    c.beginPath();
    c.moveTo(sx, sy);
    c.lineTo(ex, ey);
    c.lineWidth = 2;
    c.strokeStyle = '#d9534f';
    c.stroke();

    // arrowhead
    const angle = Math.atan2(ey - sy, ex - sx);
    const ah = 6; // arrowhead size
    c.beginPath();
    c.moveTo(ex, ey);
    c.lineTo(ex - ah * Math.cos(angle - Math.PI / 6), ey - ah * Math.sin(angle - Math.PI / 6));
    c.lineTo(ex - ah * Math.cos(angle + Math.PI / 6), ey - ah * Math.sin(angle + Math.PI / 6));
    c.closePath();
    c.fillStyle = '#d9534f';
    c.fill();
  }







}