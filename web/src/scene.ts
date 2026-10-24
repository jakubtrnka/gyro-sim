import * as T from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { TransformControls } from "three/addons/controls/TransformControls.js";
import type { Preset, Quat, Snapshot } from "../generated/web/gyro_wasm";
import { spinIndex } from "./display";

const X = new T.Vector3(1, 0, 0),
  Y = new T.Vector3(0, 1, 0),
  Z = new T.Vector3(0, 0, 1);
const COLOR = {
  housing: 0x607b90,
  outer: 0xb8784f,
  inner: 0x3a9e9a,
  rotor: 0xd5af59,
};
// The ONLY component-pose conversion: aircraft (x,y,z) -> scene (y,-z,-x).
const conversion = new T.Quaternion().setFromRotationMatrix(
  new T.Matrix4().set(0, 1, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, 1),
);

function material(color: number, opacity = 1) {
  return new T.MeshStandardMaterial({
    color,
    metalness: 0.35,
    roughness: 0.42,
    transparent: opacity < 1,
    opacity,
    depthWrite: opacity === 1,
  });
}
function cylinder(
  radius: number,
  height: number,
  axis: T.Vector3,
  mat: T.Material,
) {
  const mesh = new T.Mesh(
    new T.CylinderGeometry(radius, radius, height, 64),
    mat,
  );
  mesh.quaternion.setFromUnitVectors(Y, axis);
  return mesh;
}
function ring(
  radius: number,
  thickness: number,
  normal: T.Vector3,
  color: number,
) {
  const mesh = new T.Mesh(
    new T.TorusGeometry(radius, thickness, 12, 100),
    material(color),
  );
  mesh.quaternion.setFromUnitVectors(Z, normal);
  return mesh;
}
function disposeGroup(group: T.Group) {
  group.traverse((object) => {
    if (object instanceof T.Mesh || object instanceof T.LineSegments) {
      object.geometry.dispose();
      for (const mat of Array.isArray(object.material)
        ? object.material
        : [object.material])
        mat.dispose();
    }
  });
  group.clear();
}

export class MechanismView {
  readonly renderer: T.WebGLRenderer;
  readonly camera = new T.PerspectiveCamera(38, 1, 0.1, 100);
  readonly scene = new T.Scene();
  readonly root = new T.Group();
  readonly housing = new T.Group();
  readonly outer = new T.Group();
  readonly inner = new T.Group();
  readonly rotor = new T.Group();
  readonly target = new T.Group();
  readonly triad = document.createElement("div");
  readonly orbit: OrbitControls;
  readonly transform: TransformControls;
  readonly momentum = new T.ArrowHelper(
    Y,
    new T.Vector3(),
    1,
    0x668b42,
    0.18,
    0.1,
  );
  readonly torque = new T.ArrowHelper(
    X,
    new T.Vector3(),
    1,
    0x9f5f89,
    0.18,
    0.1,
  );
  readonly axes = new T.Group();
  private axesLocal: T.Vector3[] = [X, Y, Z];
  private jointArrows: T.ArrowHelper[] = [];
  private moveMode = true;
  private fixedHousing = false;
  private arcball: {
    start: T.Vector3;
    orientation: T.Quaternion;
    pointer: number;
  } | null = null;
  private marker = true;
  private transparent = true;
  private overlays = false;
  private observer: ResizeObserver;
  private onTarget: (q: Quat) => void;
  private onFailure: (message: string) => void;

  constructor(
    private container: HTMLElement,
    onTarget: (q: Quat) => void,
    onFailure: (message: string) => void,
  ) {
    this.onTarget = onTarget;
    this.onFailure = onFailure;
    this.renderer = new T.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setClearColor(0xedf1ed);
    this.renderer.outputColorSpace = T.SRGBColorSpace;
    this.renderer.domElement.setAttribute("aria-hidden", "true");
    container.prepend(this.renderer.domElement);
    this.triad.style.cssText =
      "position:absolute;right:16px;bottom:40px;width:90px;height:95px;pointer-events:none";
    this.triad.setAttribute("aria-label", "World reference axes");
    container.append(this.triad);
    this.root.quaternion.copy(conversion);
    this.scene.add(this.root);
    this.root.add(
      this.housing,
      this.outer,
      this.inner,
      this.rotor,
      this.target,
      this.axes,
      this.momentum,
      this.torque,
    );
    const ambient = new T.HemisphereLight(0xffffff, 0xa3b4a5, 2.7);
    const key = new T.DirectionalLight(0xffffff, 3.4);
    key.position.set(3, 6, 4);
    const fill = new T.DirectionalLight(0xffebc8, 1.5);
    fill.position.set(-3, 1, -3);
    this.scene.add(ambient, key, fill);
    const grid = new T.GridHelper(18, 36, 0xb3c2b4, 0xd6dfd4);
    grid.position.y = -2.4;
    (grid.material as T.Material).transparent = true;
    (grid.material as T.Material).opacity = 0.5;
    this.scene.add(grid);
    this.camera.position.set(4.8, 3.5, 6.5);
    this.camera.lookAt(0, 0, 0);
    this.orbit = new OrbitControls(this.camera, this.renderer.domElement);
    this.orbit.enableDamping = false;
    this.orbit.enablePan = false;
    this.orbit.minDistance = 5;
    this.orbit.maxDistance = 15;
    this.orbit.enableRotate = false;
    this.transform = new TransformControls(
      this.camera,
      this.renderer.domElement,
    );
    this.transform.setMode("rotate");
    this.transform.setSpace("local");
    this.transform.setSize(0.8);
    this.transform.attach(this.target);
    this.scene.add(this.transform.getHelper());
    this.transform.addEventListener("objectChange", () =>
      this.onTarget(this.target.quaternion.toArray() as Quat),
    );
    this.transform.addEventListener("dragging-changed", (event) => {
      this.orbit.enableRotate =
        !event.value && !this.moveMode && !this.fixedHousing;
    });
    const canvas = this.renderer.domElement;
    canvas.addEventListener("pointerdown", (e) => {
      if (!this.moveMode || e.button !== 0 || this.transform.axis !== null)
        return;
      this.arcball = {
        start: this.ball(e),
        orientation: this.target.quaternion.clone(),
        pointer: e.pointerId,
      };
      canvas.setPointerCapture(e.pointerId);
    });
    canvas.addEventListener("pointermove", (e) => {
      if (!this.arcball || this.arcball.pointer !== e.pointerId) return;
      const delta = new T.Quaternion().setFromUnitVectors(
        this.arcball.start,
        this.ball(e),
      );
      // Input rotation changes basis (unlike component poses): camera -> scene -> aircraft.
      delta
        .premultiply(this.camera.quaternion)
        .multiply(this.camera.quaternion.clone().invert());
      delta.premultiply(conversion.clone().invert()).multiply(conversion);
      this.target.quaternion
        .copy(delta.multiply(this.arcball.orientation))
        .normalize();
      this.onTarget(this.target.quaternion.toArray() as Quat);
    });
    const end = () => {
      this.arcball = null;
    };
    canvas.addEventListener("pointerup", end);
    canvas.addEventListener("pointercancel", end);
    canvas.addEventListener("lostpointercapture", end);
    canvas.addEventListener("webglcontextlost", (e) => {
      e.preventDefault();
      this.onFailure(
        "Graphics context lost. Reload to reinitialize the view; simulation paused.",
      );
    });
    this.observer = new ResizeObserver(() => this.resize());
    this.observer.observe(container);
    this.resize();
  }
  private ball(e: PointerEvent) {
    const box = this.renderer.domElement.getBoundingClientRect();
    const scale = Math.min(box.width, box.height);
    const x = (2 * (e.clientX - box.left) - box.width) / scale;
    const y = (box.height - 2 * (e.clientY - box.top)) / scale;
    return new T.Vector3(
      x,
      y,
      Math.sqrt(Math.max(0, 1 - x * x - y * y)),
    ).normalize();
  }
  private resize() {
    const { width, height } = this.container.getBoundingClientRect();
    if (!width || !height) return;
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);
  }
  configure(preset: Preset) {
    for (const group of [
      this.housing,
      this.outer,
      this.inner,
      this.rotor,
      this.target,
      this.axes,
    ])
      disposeGroup(group);
    this.axesLocal =
      preset === "Directional"
        ? [Z, X, Y]
        : preset === "Attitude"
          ? [X, Y, Z]
          : [X, Y];
    const [a, b, c] = this.axesLocal;
    const spin = c ?? b;
    const frame = new T.EdgesGeometry(new T.BoxGeometry(3.9, 3.9, 3.9));
    const line = new T.LineSegments(
      frame,
      new T.LineBasicMaterial({
        color: COLOR.housing,
        transparent: true,
        opacity: 0.55,
      }),
    );
    this.housing.add(line);
    // Four open structural rails give the housing visual depth without hiding the rings.
    for (const y of [-1.95, 1.95])
      for (const z of [-1.95, 1.95]) {
        const rail = new T.Mesh(
          new T.BoxGeometry(3.95, 0.08, 0.08),
          material(COLOR.housing, 0.55),
        );
        rail.position.set(0, y, z);
        this.housing.add(rail);
      }
    const ghost = new T.LineSegments(
      new T.EdgesGeometry(new T.BoxGeometry(4.02, 4.02, 4.02)),
      new T.LineDashedMaterial({
        color: 0x699480,
        transparent: true,
        opacity: 0.45,
        dashSize: 0.1,
        gapSize: 0.08,
      }),
    );
    ghost.computeLineDistances();
    this.target.add(ghost);
    this.outer.add(
      ring(1.68, 0.085, new T.Vector3().crossVectors(a, b), COLOR.outer),
    );
    if (c)
      this.inner.add(
        ring(1.35, 0.075, new T.Vector3().crossVectors(b, c), COLOR.inner),
      );
    const bearing = (
      parent: T.Group,
      axis: T.Vector3,
      distance: number,
      color: number,
    ) => {
      for (const sign of [-1, 1]) {
        const pin = cylinder(0.11, 0.32, axis, material(color));
        pin.position.copy(axis).multiplyScalar(sign * distance);
        parent.add(pin);
      }
    };
    bearing(this.housing, a, 1.8, COLOR.housing);
    bearing(this.outer, b, c ? 1.5 : 1.2, COLOR.outer);
    if (c) bearing(this.inner, c, 1.1, COLOR.inner);
    this.rotor.add(cylinder(0.94, 0.22, spin, material(COLOR.rotor)));
    this.rotor.add(cylinder(0.095, 2.15, spin, material(0x7d8685)));
    const marker = new T.Group();
    marker.name = "marker";
    const basis = Math.abs(spin.dot(X)) < 0.5 ? X : Y;
    for (const sign of [-1, 1]) {
      const stripe = new T.Mesh(
        new T.BoxGeometry(0.76, 0.025, 0.035),
        material(0xfff3cb),
      );
      // Build a radial marker in the plane perpendicular to the spindle.
      stripe.quaternion.setFromUnitVectors(X, basis);
      stripe.position
        .copy(basis)
        .multiplyScalar(0.47)
        .addScaledVector(spin, sign * 0.115);
      marker.add(stripe);
    }
    this.rotor.add(marker);
    this.jointArrows = this.axesLocal.map((_, i) => {
      const arrow = new T.ArrowHelper(
        X,
        new T.Vector3(),
        2.25 - i * 0.15,
        [COLOR.outer, COLOR.inner, COLOR.rotor][i],
        0.12,
        0.07,
      );
      this.axes.add(arrow);
      return arrow;
    });
    this.target.quaternion.identity();
    this.setOptions(this.transparent, this.overlays, this.marker);
  }
  setOptions(transparent: boolean, vectors: boolean, marker: boolean) {
    this.transparent = transparent;
    this.overlays = vectors;
    this.marker = marker;
    this.housing.traverse((object) => {
      if (object instanceof T.Mesh || object instanceof T.LineSegments) {
        const mat = object.material as T.Material;
        mat.transparent = transparent;
        mat.opacity = transparent ? 0.5 : 1;
        mat.depthWrite = !transparent;
      }
    });
    this.momentum.visible = vectors;
    this.torque.visible = vectors;
    const mesh = this.rotor.getObjectByName("marker");
    if (mesh) mesh.visible = marker;
  }
  setTarget(q: Quat) {
    if (!this.transform.dragging && !this.arcball)
      this.target.quaternion.fromArray(q).normalize();
  }
  setMode(move: boolean) {
    this.arcball = null;
    this.moveMode = move;
    this.transform.enabled = move;
    this.transform.getHelper().visible = move;
    this.target.visible = move;
    this.orbit.enableRotate = !move && !this.fixedHousing;
  }
  setCameraView(fixed: boolean) {
    this.fixedHousing = fixed;
    this.orbit.enableRotate = !this.moveMode && !fixed;
    if (!fixed) this.home();
  }
  home() {
    this.camera.position.set(4.8, 3.5, 6.5);
    this.orbit.target.set(0, 0, 0);
    this.camera.lookAt(0, 0, 0);
    this.orbit.update();
  }
  render(s: Snapshot) {
    this.housing.quaternion.fromArray(s.housing.orientation);
    this.outer.quaternion.fromArray(s.component_orientations[0]);
    this.inner.visible = s.preset !== "TurnIndicator";
    if (this.inner.visible)
      this.inner.quaternion.fromArray(s.component_orientations[1]);
    const index = spinIndex(s);
    const spin = this.axesLocal[index];
    // Remove real spin phase from the symmetric disc, adding only the labelled visual marker phase.
    this.rotor.quaternion
      .fromArray(s.component_orientations[index])
      .multiply(
        new T.Quaternion().setFromAxisAngle(
          spin,
          -s.joints.angles[index] +
            (this.marker ? s.joints.angles[index] / 200 : 0),
        ),
      );
    for (let i = 0; i < this.jointArrows.length; i++) {
      const parent = new T.Quaternion(
        ...(i === 0 ? s.housing.orientation : s.component_orientations[i - 1]),
      );
      this.jointArrows[i].setDirection(
        this.axesLocal[i].clone().applyQuaternion(parent),
      );
    }
    for (const [arrow, vector, scale] of [
      [this.momentum, s.diagnostics.rotor_angular_momentum, 30],
      [this.torque, s.diagnostics.housing_torque, 150],
    ] as const) {
      const v = new T.Vector3(...vector);
      const length = v.length() * scale;
      arrow.visible = this.overlays && length > 1e-9;
      if (length > 1e-9) {
        arrow.setDirection(v.normalize());
        arrow.setLength(
          length,
          Math.min(0.2, length * 0.2),
          Math.min(0.1, length * 0.1),
        );
      }
    }
    if (this.fixedHousing) {
      const distance = this.camera.position.length();
      const pose = conversion
        .clone()
        .multiply(new T.Quaternion(...s.housing.orientation))
        .multiply(conversion.clone().invert());
      this.camera.position
        .set(4.8, 3.5, 6.5)
        .normalize()
        .multiplyScalar(distance)
        .applyQuaternion(pose);
      this.camera.up.copy(Y).applyQuaternion(pose);
      this.camera.lookAt(0, 0, 0);
    } else {
      this.camera.up.copy(Y);
      this.orbit.update();
    }
    const cameraFromModel = this.camera.quaternion
      .clone()
      .invert()
      .multiply(conversion);
    this.triad.innerHTML = `<svg viewBox="0 0 90 95" aria-hidden="true">${[
      [X, "x"],
      [Y, "y"],
      [Z, "z"],
    ]
      .map(([axis, name]) => {
        const v = (axis as T.Vector3).clone().applyQuaternion(cameraFromModel);
        return `<path d="M45 40L${45 + v.x * 29} ${40 - v.y * 29}" stroke="#59776b" stroke-width="1.5"/><text x="${45 + v.x * 38}" y="${44 - v.y * 38}" text-anchor="middle" fill="#426158" font-size="10">${name}</text>`;
      })
      .join(
        "",
      )}<text x="45" y="92" text-anchor="middle" fill="#71877b" font-size="8">WORLD AXES</text></svg>`;
    this.renderer.render(this.scene, this.camera);
  }
  dispose() {
    this.observer.disconnect();
    this.transform.dispose();
    this.orbit.dispose();
    this.renderer.dispose();
    this.triad.remove();
  }
}
