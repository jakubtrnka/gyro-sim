/** Wall time only decides how many fixed Rust steps to request. It never alters dt. */
export class FixedClock {
  running = false;
  speed = 1;
  accumulator = 0;
  last: number | null = null;
  readonly dt: number;
  constructor(dt: number) {
    if (!Number.isFinite(dt) || dt <= 0)
      throw new Error("Invalid physics step");
    this.dt = dt;
  }
  start() {
    this.running = true;
    this.last = null;
    this.accumulator = 0;
  }
  pause() {
    this.running = false;
    this.last = null;
    this.accumulator = 0;
  }
  tick(
    now: number,
    advance: (count: number) => void,
  ): { steps: number; overloaded: boolean } {
    if (!this.running) return { steps: 0, overloaded: false };
    if (this.last === null) {
      this.last = now;
      return { steps: 0, overloaded: false };
    }
    const elapsed = (now - this.last) / 1000;
    this.last = now;
    this.accumulator += elapsed * this.speed;
    if (
      !Number.isFinite(this.accumulator) ||
      elapsed < 0 ||
      this.accumulator > 0.25
    ) {
      this.pause(); // discard wall-clock debt, never physical steps
      return { steps: 0, overloaded: true };
    }
    const steps = Math.min(2048, Math.floor(this.accumulator / this.dt + 1e-9));
    if (steps > 0) {
      advance(steps);
      this.accumulator = Math.max(0, this.accumulator - steps * this.dt);
    }
    return { steps, overloaded: false };
  }
  get fraction() {
    return Math.min(1, Math.max(0, this.accumulator / this.dt));
  }
}
