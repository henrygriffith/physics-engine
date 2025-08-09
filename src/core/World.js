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
    const g = this.Gravity();

    // Integrate
    for (const b of this._bodies) {
      if (b.IsFixed() || b.InverseMass() === 0) {
        b.ClearAllForces();
        continue;
      }

      const invm = b.InverseMass();
      const F = b.Force(); // clone of external forces accumulated this frame.
      const V = b.Velocity();
      const P = b.Position();

      if (!Number.isFinite(invm)) { console.warn("Bad invMass", invm, b); continue; }
      if (!P || !V || !F) { console.warn("Missing vectors", { P, V, F, b }); continue; }

      const ax = g.x + F.x * invm;
      const ay = g.y + F.y * invm;

      const vx = V.x + ax * dt;
      const vy = V.y + ay * dt;
      b.SetVelocityXY(vx, vy); // update velocity.

      const px = P.x + vx * dt;
      const py = P.y + vy * dt;
      b.SetPositionXY(px, py); // move using *new* velocity (semi-implicit Euler).

      b.ClearAllForces();
    }

    // Collisions (naive all-pairs, circles only for now)
    // all bodies are checked against eachother (O(n^2)).
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

    // World bounds. After pairwise collisions, clamp each body to the axis-aligned box
    // and flip velocity components with restitution when it hits a wall.
    if (this.Bounds()) {
      const { minX, minY, maxX, maxY } = this.Bounds();
      for (const b of this.Bodies()) {
        if (b.IsFixed() || b.InverseMass() === 0)
          continue;

        const P = b.Position();
        const V = b.Velocity();
        const r = b.Radius();
        const e = b.Restitution();

        if (P.x - r < minX) {
          b.SetPositionXY(minX + r, P.y);
          b.SetVelocityXY(-V.x * e, V.y);
        } else if (P.x + r > maxX) {
          b.SetPositionXY(maxX - r, P.y);
          b.SetVelocityXY(-V.x * e, V.y);
        }

        const P2 = b.Position();
        const V2 = b.Velocity();
        if (P2.y - r < minY) {
          b.SetPositionXY(P2.x, minY + r);
          b.SetVelocityXY(V2.x, -V2.y * e);
        } else if (P2.y + r > maxY) {
          b.SetPositionXY(P2.x, maxY - r);
          b.SetVelocityXY(V2.x, -V2.y * e);
        }
      }
    }
  }

  // Broadphase: the first stage of collision detection. quickly figure out which object
  // pairs might be colliding. We should toss pairs that are obviously far apart.

  // We'll need a simple bounding volume (usually an Axis-Aligned Bounding Box, AABB).
  // We'll need simple cheap test to see if overlap.
  // Only send overlapping AABB pairs to narrowphase for accurate collision math.

  _ResolveCircleCircle(A, B) { // if two circles overlap, push them apart and apply a bounce impulse.
    // Fixed bodies can still be hit, but won't move.
    const invA = A.InverseMass();
    const invB = B.InverseMass();
    if (invA === 0 && invB === 0) // if both fixed, do nothing.
      return;

    const PA = A.Position();
    const PB = B.Position();
    const rA = A.Radius();
    const rB = B.Radius();

    const dx = PB.x - PA.x;
    const dy = PB.y - PA.y;
    const distSq = dx * dx + dy * dy;
    const r = rA + rB;

    if (distSq <= 0) {
      // same pos; nudge apart along arbitrary axis to avoid NaN normals
      const nudge = 0.5 * r;
      if (invA > 0) A.SetPositionXY(PA.x - nudge, PA.y);
      if (invB > 0) B.SetPositionXY(PB.x + nudge, PB.y);
      return;
    }

    if (distSq >= r * r) // no overlap;
      return;

    const dist = Math.sqrt(distSq);
    const nx = dx / dist; // unit normal x
    const ny = dy / dist; // unit normal y
    const penetration = r - dist; // how much they overlap;

    // Positional correction
    // split the separation by inverse mass: lighter objects move more and fixed don't move.
    const invSum = invA + invB;
    if (invSum > 0) {
      const corr = penetration / invSum;
      if (invA > 0) A.SetPositionXY(PA.x - nx * corr * invA, PA.y - ny * corr * invA);
      if (invB > 0) B.SetPositionXY(PB.x - nx * corr * invB, PB.y - ny * corr * invB);
    }

    // Relative velocity along normal
    const VA = A.Velocity();
    const VB = B.Velocity();
    const rvx = VB.x - VA.x;
    const rvy = VB.y - VA.y;
    const velAlongNormal = rvx * nx + rvy * ny;
    if (velAlongNormal > 0) // moving apart already -> no bounce.
      return;

    const e = Math.min(A.Restitution(), B.Restitution()); // bounciness of contact.
    const j = (-(1 + e) * velAlongNormal) / (invSum || 1); // scalar impulse magnitude.

    const impX = nx * j; // impulses
    const impY = ny * j;

    // apply impulses opposite/along the normal, scaled by inverse mass.
    if (invA > 0) A.SetVelocityXY(VA.x - impX * invA, VA.y - impY * invA);
    if (invB > 0) B.SetVelocityXY(VB.x + impX * invB, VB.y + impY * invB);


    // If two balls are moving into each other along n, the impulse cancels
    // that normal component and adds a bounce based on 'e'.
    // Tangential (sideways) velocity is unchanged here becacuse we haven't added friction yet.

  }

}