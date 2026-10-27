import { Quaternion, Vector3 } from "three";
import type { Quat } from "../generated/web/gyro_wasm";

type Point = [number, number, number];
type Face = { vertices: Point[]; color: string };
const faces: Face[] = [];

// A tiny solid aircraft in model coordinates: x nose, y right wing, z underside.
// Projected as SVG polygons: no second WebGL context or external model asset.
function slab(outline: [number, number][], top: number, bottom: number) {
  const upper = outline.map(([x, y]): Point => [x, y, top]);
  const lower = outline.map(([x, y]): Point => [x, y, bottom]);
  faces.push({ vertices: upper, color: "#edf4f0" });
  faces.push({ vertices: lower, color: "#b67b46" });
  for (let i = 0; i < outline.length; i++) {
    const next = (i + 1) % outline.length;
    faces.push({
      vertices: [upper[i], upper[next], lower[next], lower[i]],
      color: "#66817f",
    });
  }
}
slab(
  [
    [0.45, 0],
    [0, 1.42],
    [-0.34, 1.42],
    [-0.23, 0],
    [-0.34, -1.42],
    [0, -1.42],
  ],
  -0.04,
  0.04,
);
slab(
  [
    [-0.85, 0],
    [-1.05, 0.65],
    [-1.3, 0.65],
    [-1.19, 0],
    [-1.3, -0.65],
    [-1.05, -0.65],
  ],
  -0.08,
  0.02,
);
slab(
  [
    [1.55, 0],
    [0.65, 0.18],
    [-0.55, 0.16],
    [-1.35, 0],
    [-0.55, -0.16],
    [0.65, -0.18],
  ],
  -0.16,
  0.12,
);
faces.push({
  vertices: [
    [-0.65, 0, -0.16],
    [-1.08, 0, -0.68],
    [-1.35, 0, -0.16],
  ],
  color: "#327c7d",
});

export function aircraftProjection(housing: Quat, cameraFromWorld: Quaternion) {
  const orientation = cameraFromWorld
    .clone()
    .multiply(new Quaternion(...housing));
  const view = (point: Point) =>
    new Vector3(...point).applyQuaternion(orientation);
  return {
    faces: faces
      .map((face) => {
        const vertices = face.vertices.map(view);
        return {
          color: face.color,
          vertices,
          depth: vertices.reduce((sum, v) => sum + v.z, 0) / vertices.length,
        };
      })
      .sort((a, b) => a.depth - b.depth),
    nose: view([1.55, 0, 0]),
    worldUp: new Vector3(0, 0, -1).applyQuaternion(cameraFromWorld),
    topAlignment: new Vector3(0, 0, -1)
      .applyQuaternion(new Quaternion(...housing))
      .dot(new Vector3(0, 0, -1)),
  };
}

export function aircraftSvg(
  housing: Quat,
  cameraFromWorld: Quaternion,
): string {
  const model = aircraftProjection(housing, cameraFromWorld);
  const xy = (v: Vector3) =>
    `${(80 + v.x * 31).toFixed(2)},${(66 - v.y * 31).toFixed(2)}`;
  const horizon = Array.from({ length: 49 }, (_, i) => {
    const angle = (i * 2 * Math.PI) / 48;
    return xy(
      new Vector3(
        1.7 * Math.cos(angle),
        1.7 * Math.sin(angle),
        0,
      ).applyQuaternion(cameraFromWorld),
    );
  }).join(" ");
  const up = model.worldUp;
  const upDirection =
    Math.hypot(up.x, up.y) < 0.12 ? (up.z > 0 ? "towards" : "away") : "side";
  const upSymbol =
    Math.hypot(up.x, up.y) < 0.12
      ? `<circle cx="20" cy="22" r="6" fill="none" stroke="#4b7161"/>${up.z > 0 ? '<circle cx="20" cy="22" r="2" fill="#4b7161"/>' : '<path d="M16 18L24 26M16 26L24 18" stroke="#4b7161"/>'}`
      : `<path d="M20 22L${20 + 17 * up.x} ${22 - 17 * up.y}" stroke="#4b7161" stroke-width="1.6" marker-end="url(#world-up-head)"/>`;
  const top =
    model.topAlignment < -0.05
      ? "Top faces down"
      : model.topAlignment > 0.05
        ? "Top faces up"
        : "Top is vertical";
  return `<svg viewBox="0 0 160 128" role="img" aria-label="Aircraft orientation: ${top.toLowerCase()}. Light upper surface, ochre underside.">
    <defs><marker id="world-up-head" viewBox="0 0 6 6" refX="5" refY="3" markerWidth="5" markerHeight="5" orient="auto-start-reverse"><path d="M0 0L6 3L0 6" fill="#4b7161"/></marker></defs>
    <polyline points="${horizon}" fill="none" stroke="#c6d5cc" stroke-dasharray="3 3"/>
    <g data-aircraft="true" stroke="#405d65" stroke-width="0.8" stroke-linejoin="round">${model.faces.map((face) => `<polygon points="${face.vertices.map(xy).join(" ")}" fill="${face.color}"/>`).join("")}</g>
    <circle cx="${80 + model.nose.x * 31}" cy="${66 - model.nose.y * 31}" r="2.6" fill="#b77930"/>
    <text x="${80 + model.nose.x * 36}" y="${69 - model.nose.y * 36}" text-anchor="middle" fill="#87602b" font-size="8">FWD</text>
    <g data-world-up="${upDirection}">${upSymbol}<text x="20" y="47" text-anchor="middle" fill="#4b7161" font-size="7">WORLD UP</text></g>
    <text x="80" y="119" text-anchor="middle" fill="#4c6768" font-size="9">${top}</text>
  </svg>`;
}
