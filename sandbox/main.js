
import { World } from '../src/core/World.js';
import { Body } from '../src/core/Body.js';
import { Vec2 } from '../src/math/Vec2.js';
import { CanvasRenderer } from "../src/render/CanvasRenderer.js";

const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');

// HiDPI scaling for crisp rendering
function SetupHiDPI() {
  const dpr = window.devicePixelRatio || 1;
  const cssWidth = 800;
  const cssHeight = 500;

  // ensure attributes exist as numbers
  canvas.width = Math.round(cssWidth * dpr);
  canvas.height = Math.round(cssHeight * dpr);
  canvas.style.width = cssWidth + 'px';
  canvas.style.height = cssHeight + 'px';

  ctx.setTransform(dpr, 0, 0, dpr, 0, 0); // draw in CSS pixels;
}
SetupHiDPI();

// WORLD + RENDERER
const renderer = new CanvasRenderer(canvas, { showVelocity: true, velocityScale: 0.03 });
const world = new World({ gravity: new Vec2(0, 800), solverIterations: 2 });

// bounds in CSS pixels (since we set transform);
function UpdateBounds() {
  const cssW = parseInt(canvas.style.width, 10);
  const cssH = parseInt(canvas.style.height, 10);
  world.SetBounds({ minX: 0, minY: 0, maxX: cssW, maxY: cssH });
}
UpdateBounds();

window.addEventListener('resize', () => {
  SetupHiDPI();
  UpdateBounds();
});

// Seed some bodies
function SpawnBall(x, y, vx = 0, vy = 0, r = 14 + Math.random() * 10) {
  const b = new Body({
    position: new Vec2(x, y),
    velocity: new Vec2(vx, vy),
    mass: 1,
    radius: r,
    restitution: 0.82
  });
  world.AddBody(b);
  return b;
}

for (let i = 0; i < 5; i++)
  for (let j = 0; j < 4; j++)
    SpawnBall(80 + i * 60, 40 + j * 40, (Math.random() - 0.5) * 60, 0);

canvas.addEventListener('pointerdown', (e) => {
  const rect = canvas.getBoundingClientRect();
  const cssX = e.clientX - rect.left;
  const cssY = e.clientY - rect.top;
  SpawnBall(cssX, cssY, (Math.random() - 0.5) * 180, -220);
});

// let last = performance.now();
// let paused = false;

// function Loop(t) {
//   const dt = Math.min((t - last) / 1000, 0.033) // seconds, clamp ~30 FPS max step.
//   last = t;

//   if (!paused)
//     world.Step(dt);

//   renderer.Clear();
//   renderer.DrawBodies(world.Bodies());

//   requestAnimationFrame(Loop);
// }

// requestAnimationFrame(Loop);

window.addEventListener('keydown', (e) => {
  if (e.key.toLowerCase() === 'p')
    paused = !paused;
})

window.addEventListener('keydown', (e) => {
  if (e.key.toLowerCase() === 'v') {
    renderer.SetDebugOptions({ showVelocity: !renderer._showVelocity });
  }
});



let last = performance.now();
let frames = 0;

function Loop(t) {
  const dt = Math.min((t - last) / 1000, 0.033);
  last = t;

  // 0) prove RAF is running
  if ((frames++ % 60) === 0) console.log("tick dt=", dt.toFixed(3));

  // 1) update physics
  world.Step(dt);

  // 2) clear
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // 3A) HARD-CODED circle (should ALWAYS show)
  ctx.beginPath();
  ctx.arc(100, 80, 12, 0, Math.PI * 2);
  ctx.fillStyle = "red";
  ctx.fill();

  // 3B) DRAW USING BODY FIELDS (bypass getters)
  const arr = world.Bodies();
  const b0 = arr[0];
  if (!b0) console.warn("no bodies!");
  else {
    const p = b0._position || b0.position || (b0.Position && b0.Position());
    const r = b0._radius || b0.radius || (b0.Radius && b0.Radius());
    console.log("p,r =", p, r);

    // draw lime if numbers look sane
    if (p && Number.isFinite(p.x) && Number.isFinite(p.y) && Number.isFinite(r) && r > 0) {
      ctx.beginPath();
      ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
      ctx.fillStyle = "lime";
      ctx.fill();
    } else {
      console.warn("bad position/radius", { p, r });
    }
  }

  // 3C) DRAW VIA RENDERER (if the two above show, but this doesn't, renderer is the bug)
  renderer.Clear();
  renderer.DrawBodies(world.Bodies());

  requestAnimationFrame(Loop);
}
requestAnimationFrame(Loop);