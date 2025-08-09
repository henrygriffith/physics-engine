

export default class Vec2 {

  static FromObject(obj)
    { return new Vec2(obj.x, obj.y); }
  static AddVectors(a, b)
    { return new Vec2(a.x + b.x, a.y + b.y); }
  static SubtractVectors(a, b)
    { return new Vec2(a.x - b.x, a.y - b.y); }

  constructor(x = 0, y = 0) {
    this.x = x;
    this.y = y;
  }

  Clone()
    { return new Vec2(this.x, this.y); }

  SetX(x)
    { this.x = x; return this; }
  SetY(y)
    { this.y = y; return this; }
  SetXY(x, y)
    { this.SetX(x); this.SetY(y); return this; }

  AddVector(v)
    { this.x += v.x; this.y += v.y; return this; }
  SubtractVector(v)
    { this.x -= v.x; this.y -= v.y; return this; }

  ScaleBy(scalar)
    { this.x *= scalar; this.y *= scalar; }

  DotProduct(v)
    { return this.x * v.x + this.y * v.y; }

  Length()
    { return Math.hypot(this.x, this.y); }

  Normalize() {
    const len = this.Length() || 1;
    this.x /= len; this.y /= len;
    return this;
  }
}