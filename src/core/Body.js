
import { Vec2 } from "../math/Vec2.js";

export class Body {
  constructor({
    position = new Vec2(),
    velocity = new Vec2(),
    mass = 1,
    radius = 10, // simple circle body for now;
    restitution = 0.6, // coefficient of bounciness. [0, 1]
    fixed = false
  }) {
    this._position = Vec2.FromObject(position);
    this._velocity = Vec2.FromObject(velocity);
    this._force = new Vec2();

    this._mass = mass;
    this._inverseMass = (fixed || mass === 0) ? 0 : 1 / mass;

    this._radius = radius;
    this._restitution = restitution;
    this._fixed = fixed;
  }

  // ------------------
  // Position
  Position()
    { return this._position.Clone(); }
  SetPositionVec(vec)
    { this._position = Vec2.FromObject(vec); return this; }
  SetPositionXY(x, y)
    { this._position.SetXY(x, y); return this; }

  // ------------------
  // Velocity
  Velocity()
    { return this._velocity.Clone(); }
  SetVelocityVec(vec)
    { this._velocity = Vec2.FromObject(vec); return this; }
  SetVelocityXY(x, y)
    { this._velocity.SetXY(x, y); return this; }

  // ------------------
  // Force
  Force()
    { return this._force.Clone(); }
  SetForceVec(vec)
    { this._force = Vec2.FromObject(vec); }
  SetForceXY(x, y)
    { this._force.SetXY(x, y); return this; }
  ApplyForceVec(vec)
    { this._force.AddVector(vec); }
  ClearAllForces()
    { this._force.setXY(0, 0); return this; }

  // ------------------
  // Mass
  Mass()
    { return this._mass; }
  SetMass(mass) {
    if (mass <= 0) {
      this._mass = 0;
      this._inverseMass = 0;
      this._fixed = true;
    } else {
      this._mass = mass;
      this._inverseMass = 1 / mass;
      this._fixed = false;
    }
    return this;
  }
  InverseMass()
    { return this._inverseMass; }

  // ------------------
  // Radius
  Radius()
    { return this._radius; }
  SetRadius(r) {
    if (r <= 0) throw new Error("Radius must be positive.");
    this._radius = r; return this;
  }

  // ------------------
  // Restitution
  Restitution()
    { return this._restitution; }
  SetRestitution(e)
    { this._restitution = Math.max(0, Math.min(1, e)); } // clamp [0, 1]

  // ------------------
  // Fixed State
  IsFixed()
    { return this._fixed; }
  SetFixed() {
    this._fixed = !!this._fixed;
    if (fixed) this._inverseMass = 0;
    else this._inverseMass = this._mass > 0 ? 1 / this._mass : 0;
    return this;
  }

// End of class
}