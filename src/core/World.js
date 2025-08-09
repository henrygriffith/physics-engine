import { Vec2 } from "../math/Vec2.js";

export class World {
  constructor({
    gravity = new Vec2(0, 800),
    solverIterations = 2,
    bounds = null, // { minX, minY, maxX, maxY } or null;
  } = {}) {
    this._gravity = Vec2.FromObject(gravity);
    this._solverIterations = solverIterations;
    this._bodies = [];
    this._bounds = bounds;
  }

  // --------------------
  // Bodies
  AddBody(body)
    { this._bodies.push(body); return body; }
  RemoveBody(body) {
    const i = this._bodies.indexOf(body);
    if (i !== -1)
      this._bodies.splice(i, 1);

    return this;
  }
  ClearAllBodies()
    { this._bodies.length = 0; return this; }
  Bodies()
    { return [...this._bodies]; }

  // --------------------
  // Gravity
  Gravity()
    { return this._gravity.Clone(); }
  SetGravityVec(vec)
    { this._gravity = Vec2.FromObject(vec); return this; }
  SetGravityXY(x, y)
    { this._gravity.SetXY(x, y); return this; }

  // --------------------
  // Solver iterations
  SolverIterations()
    { return this._solverIterations; }
  SetSolverIterations(n)
    { this._solverIterations = Math.max(1, n | 0); return this; }

  // --------------------
  // Bounds
  Bounds()
    { return this._bounds ? { ...this._bounds } : null; }
  SetBounds(boundsOrNull)
    { this._bounds = boundsOrNull ? { ...boundsOrNull } : null; return this; }

  // Simulation step (semi-implicit Euler)
    // When we simulate motion, we integrate accel -> vel -> pos over small time steps dt.
    // If we do this naively (explicit Euler), we update pos using OLD vel,
    // which can drift and even blow up with larger time steps.

    // v = v + a * dt      // update velocity
    // p = p + v_old * dt  // update position with *old* velocity

    // v = v + a * dt      // update velocity
    // p = p + v * dt      // update position with *new* velocity

  Step(dt) {
    if (!Number.isFinite(dt) || dt <= 0) return;

    const SUBSTEPS = 2;                 // (4) substeps
    const h = dt / SUBSTEPS;
    const DAMPING = 0.02;               // mild velocity damping per second

    for (let s = 0; s < SUBSTEPS; s++) {

      // --- integrate (semi-implicit Euler) ---
      for (const b of this._bodies) {
        if (b.IsFixed() || b.InverseMass() === 0) { b.ClearAllForces(); continue; }

        const invm = b.InverseMass();
        const F = b.Force();            // copy
        const V = b.Velocity();         // copy
        const P = b.Position();         // copy

        const ax = this._gravity.x + F.x * invm;
        const ay = this._gravity.y + F.y * invm;

        // update velocity
        let vx = V.x + ax * h;
        let vy = V.y + ay * h;

        // damping (bleeds energy so things settle)
        const damp = Math.max(0, 1 - DAMPING * h);
        vx *= damp; vy *= damp;
        b.SetVelocityXY(vx, vy);

        // update position with NEW velocity (semi-implicit)
        b.SetPositionXY(P.x + vx * h, P.y + vy * h);

        b.ClearAllForces();
      }

      // --- collisions (your existing all-pairs) ---
      const N = this._bodies.length;
      for (let iter = 0; iter < this._solverIterations; iter++) {
        for (let i = 0; i < N; i++) {
          const A = this._bodies[i];
          for (let j = i + 1; j < N; j++) {
            const B = this._bodies[j];
            this._ResolveCircleCircle(A, B);
          }
        }
      }

      // --- bounds with (2) resting threshold + (3) softer walls ---
      const bounds = this._bounds;
      if (bounds) {
        const { minX, minY, maxX, maxY } = bounds;
        const REST_VEL = 5; // px/s

        for (const b of this._bodies) {
          if (b.IsFixed() || b.InverseMass() === 0) continue;

          const p = b.Position();
          const v = b.Velocity();
          const r = b.Radius();
          const wallBounce = Math.min(0.6, b.Restitution()); // (3)

          // left
          if (p.x - r < minX) {
            b.SetPositionXY(minX + r + 0.001, p.y);
            const newVx = -v.x * wallBounce;
            b.SetVelocityXY(Math.abs(newVx) < REST_VEL ? 0 : newVx, v.y);
          }
          // right
          else if (p.x + r > maxX) {
            b.SetPositionXY(maxX - r - 0.001, p.y);
            const newVx = -v.x * wallBounce;
            b.SetVelocityXY(Math.abs(newVx) < REST_VEL ? 0 : newVx, v.y);
          }

          // top
          const p2 = b.Position(), v2 = b.Velocity();
          if (p2.y - r < minY) {
            b.SetPositionXY(p2.x, minY + r + 0.001);
            const newVy = -v2.y * wallBounce;
            b.SetVelocityXY(v2.x, Math.abs(newVy) < REST_VEL ? 0 : newVy);
          }
          // bottom
          else if (p2.y + r > maxY) {
            b.SetPositionXY(p2.x, maxY - r - 0.001);
            const newVy = -v2.y * wallBounce;
            b.SetVelocityXY(v2.x, Math.abs(newVy) < REST_VEL ? 0 : newVy);
          }
        }
      }
    }
  }

  // Broadphase: the first stage of collision detection. quickly figure out which object
  // pairs might be colliding. We should toss pairs that are obviously far apart.

  // We'll need a simple bounding volume (usually an Axis-Aligned Bounding Box, AABB).
  // We'll need simple cheap test to see if overlap.
  // Only send overlapping AABB pairs to narrowphase for accurate collision math.

  _ResolveCircleCircle(A, B) {
    const invA = A.InverseMass();
    const invB = B.InverseMass();
    if (invA === 0 && invB === 0) return;

    const PA = A.Position();
    const PB = B.Position();
    const rA = A.Radius();
    const rB = B.Radius();

    let dx = PB.x - PA.x;
    let dy = PB.y - PA.y;
    const r = rA + rB;

    let distSq = dx * dx + dy * dy;
    if (distSq >= r * r) return; // no overlap

    // --- robust normal ---
    let dist = Math.sqrt(distSq);
    let nx, ny;
    if (dist < 1e-8) {
      // almost same center: pick a stable arbitrary normal
      nx = 1; ny = 0;
      dist = r; // pretend they're exactly touching
    } else {
      nx = dx / dist; ny = dy / dist;
    }

    // --- penetration correction (Baumgarte-style) ---
    const PENETRATION_SLOP = 0.01;  // ignore tiny overlaps
    const BETA = 0.2;               // 0..1, fraction to correct per pass
    const penetration = r - dist;
    const corrMag = Math.max(penetration - PENETRATION_SLOP, 0) * BETA;

    const invSum = invA + invB;
    if (corrMag > 0 && invSum > 0) {
      const corrA = corrMag * (invA / invSum);
      const corrB = corrMag * (invB / invSum);
      if (invA > 0) A.SetPositionXY(PA.x - nx * corrA, PA.y - ny * corrA);
      if (invB > 0) B.SetPositionXY(PB.x + nx * corrB, PB.y + ny * corrB);
    }

    // --- relative normal velocity & restitution ---
    const VA = A.Velocity();
    const VB = B.Velocity();
    const rvx = VB.x - VA.x;
    const rvy = VB.y - VA.y;
    const velAlongNormal = rvx * nx + rvy * ny;

    // if separating, no impulse
    if (velAlongNormal > 0) return;

    // only bounce on meaningful impacts (prevents jitter explosions)
    const REST_VEL_THRESHOLD = 20; // px/s
    const e = Math.min(A.Restitution(), B.Restitution());
    const effRest = (Math.abs(velAlongNormal) > REST_VEL_THRESHOLD) ? e : 0;

    // impulse scalar
    const denom = invSum || 1;
    let j = (-(1 + effRest) * velAlongNormal) / denom;

    // clamp rare spikes
    const MAX_IMPULSE = 1e3;
    if (j >  MAX_IMPULSE) j =  MAX_IMPULSE;
    if (j < -MAX_IMPULSE) j = -MAX_IMPULSE;

    const impX = nx * j;
    const impY = ny * j;

    if (invA > 0) A.SetVelocityXY(VA.x - impX * invA, VA.y - impY * invA);
    if (invB > 0) B.SetVelocityXY(VB.x + impX * invB, VB.y + impY * invB);
  }

}