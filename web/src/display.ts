import { Quaternion, Vector3 } from "three";
import type {
  Snapshot,
  Quat,
  Vec3,
  SimulationConfig,
  InstrumentReading,
} from "../generated/web/gyro_wasm";

export const degrees = (r: number) => (r * 180) / Math.PI;
export const radians = (d: number) => (d * Math.PI) / 180;
export const rpm = (r: number) => (r * 60) / (2 * Math.PI);
export const spinIndex = (s: { preset: string }) =>
  s.preset === "TurnIndicator" ? 1 : 2;

function slerp(a: Quat, b: Quat, t: number): Quat {
  return new Quaternion(...a).slerp(new Quaternion(...b), t).toArray() as Quat;
}

/** Display interpolation only. Never fed back into the physical state. */
export function interpolate(
  a: Snapshot,
  b: Snapshot,
  t: number,
  config: SimulationConfig,
): Snapshot {
  const lerp = (x: number, y: number) => x + (y - x) * t;
  const vec = (x: Vec3, y: Vec3) => x.map((v, i) => lerp(v, y[i])) as Vec3;
  const s = {
    ...b,
    time_seconds: lerp(a.time_seconds, b.time_seconds),
    joints: {
      angles: vec(a.joints.angles, b.joints.angles),
      rates: vec(a.joints.rates, b.joints.rates),
    },
    housing: {
      orientation: slerp(a.housing.orientation, b.housing.orientation, t),
      angular_velocity: vec(
        a.housing.angular_velocity,
        b.housing.angular_velocity,
      ),
      angular_acceleration: vec(
        a.housing.angular_acceleration,
        b.housing.angular_acceleration,
      ),
    },
    component_orientations: a.component_orientations.map((q, i) =>
      slerp(q, b.component_orientations[i], t),
    ) as [Quat, Quat, Quat],
    diagnostics: {
      ...b.diagnostics,
      rotor_angular_momentum: vec(
        a.diagnostics.rotor_angular_momentum,
        b.diagnostics.rotor_angular_momentum,
      ),
      housing_torque: vec(
        a.diagnostics.housing_torque,
        b.diagnostics.housing_torque,
      ),
    },
  };
  // Read the very same interpolated spindle drawn by the rotor mesh.
  s.diagnostics.spindle_direction = (
    s.preset === "Attitude" ? new Vector3(0, 0, 1) : new Vector3(0, 1, 0)
  )
    .applyQuaternion(new Quaternion(...s.component_orientations[spinIndex(s)]))
    .normalize()
    .toArray() as Vec3;
  s.instrument = reading(s, config);
  return s;
}

/** Geometric instrument linkage, not a second dynamics calculation. */
export function reading(
  s: Snapshot,
  config: SimulationConfig,
): InstrumentReading {
  if (s.preset === "Directional")
    return {
      Directional: {
        heading:
          (((s.heading_offset - s.joints.angles[0]) % (2 * Math.PI)) +
            2 * Math.PI) %
          (2 * Math.PI),
      },
    };
  if (s.preset === "TurnIndicator")
    return {
      TurnIndicator: {
        deflection: s.joints.angles[0],
        indicated_rate:
          (config.spring_stiffness * s.joints.angles[0]) /
          config.nominal_axial_momentum,
      },
    };
  const down = new Vector3(...s.diagnostics.spindle_direction).applyQuaternion(
    new Quaternion(...s.housing.orientation).invert(),
  );
  return {
    Attitude: {
      pitch: Math.asin(Math.min(1, Math.max(-1, -down.x))),
      bank:
        Math.hypot(down.y, down.z) < 1e-6 ? null : Math.atan2(down.y, down.z),
    },
  };
}

export function instrumentSvg(s: Snapshot): string {
  const face = s.instrument;
  let art = "";
  if ("Attitude" in face) {
    const { pitch, bank } = face.Attitude;
    if (bank === null)
      art = `<text x="120" y="115" text-anchor="middle" fill="#ffd376" font-size="13">BANK UNDEFINED</text><text x="120" y="138" text-anchor="middle" fill="#b8c7cc" font-size="10">Vertical pitch reference</text>`;
    else
      art = `<defs><clipPath id="face-clip"><circle cx="120" cy="120" r="97"/></clipPath></defs>
      <g clip-path="url(#face-clip)"><g transform="translate(120 120) rotate(${-degrees(bank ?? 0)}) translate(0 ${degrees(pitch) * 2.1})">
      <rect x="-300" y="-500" width="600" height="500" fill="#5594ac"/><rect x="-300" y="0" width="600" height="500" fill="#977752"/><path d="M-300 0H300" stroke="#fff" stroke-width="2"/>
      ${[-30, -20, -10, 10, 20, 30].map((n) => `<path d="M-${Math.abs(n) % 20 === 0 ? 26 : 17} ${-n * 2.1}H${Math.abs(n) % 20 === 0 ? 26 : 17}" stroke="white"/><text x="34" y="${-n * 2.1 + 4}" fill="white" font-size="10">${Math.abs(n)}</text>`).join("")}</g></g>
      <path d="M62 120H100L106 127H134L140 120H178M120 118V126" fill="none" stroke="#ffd376" stroke-width="4"/>
      <path d="M114 31L120 40L126 31" fill="#fff"/>`;
  } else if ("Directional" in face) {
    const heading = degrees(face.Directional.heading);
    art = `<g transform="rotate(${-heading} 120 120)">${Array.from(
      { length: 36 },
      (_, i) => {
        const d = i * 10;
        const label =
          ({ 0: "N", 9: "E", 18: "S", 27: "W" } as Record<number, string>)[i] ??
          (i % 3 === 0 ? String(d / 10) : "");
        return `<g transform="rotate(${d} 120 120)"><path d="M120 29V${i % 3 === 0 ? 43 : 36}" stroke="#b8c7cc"/><text x="120" y="59" text-anchor="middle" fill="${i === 0 ? "#ffd376" : "#eaf1ee"}" font-size="${i % 9 === 0 ? 17 : 13}">${label}</text></g>`;
      },
    ).join(
      "",
    )}</g><path d="M120 71V158M87 113L120 101L153 113M107 147H133" fill="none" stroke="#dce8e6" stroke-width="3"/><path d="M114 22L120 34L126 22" fill="#ffd376"/>`;
  } else {
    const rate = degrees(face.TurnIndicator.indicated_rate);
    art = `<text x="60" y="80" fill="#b9c9ce" text-anchor="middle">L</text><text x="180" y="80" fill="#b9c9ce" text-anchor="middle">R</text>
      <path d="M62 145V158M120 63V76M178 145V158" stroke="#c3d3d6" stroke-width="3"/>
      <g transform="rotate(${Math.max(-55, Math.min(55, (rate / 3) * 35))} 120 120)"><path d="M120 130V75M80 120H160" stroke="#ffd376" stroke-width="4"/><circle cx="120" cy="120" r="7" fill="#ffd376"/></g>
      <text x="120" y="186" fill="#b9c9ce" font-size="10" text-anchor="middle">3°/s STANDARD RATE</text>`;
  }
  return `<svg viewBox="0 0 240 240" role="img" aria-label="${s.preset} instrument"><circle cx="120" cy="120" r="117" fill="#273c46"/><circle cx="120" cy="120" r="105" fill="#142833" stroke="#42606a" stroke-width="2"/>${art}<circle cx="120" cy="120" r="98" fill="none" stroke="#0b1c25" stroke-width="3"/></svg>`;
}
