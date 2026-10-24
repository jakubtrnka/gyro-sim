//! Test-only energy oracle: second derivatives of body-frame kinetic energies.
//! Deliberately uses no production quaternion, Jacobian, inertia, or solve helpers.

use gyro_core::*;
use std::ops::{Add, Mul, Neg, Sub};

const N: usize = 7; // q[0..3], qdot[3..6], time

#[derive(Clone, Copy)]
struct D {
    v: f64,
    g: [f64; N],
    h: [[f64; N]; N],
}

impl D {
    fn constant(v: f64) -> Self {
        Self {
            v,
            g: [0.0; N],
            h: [[0.0; N]; N],
        }
    }
    fn variable(v: f64, i: usize) -> Self {
        let mut d = Self::constant(v);
        d.g[i] = 1.0;
        d
    }
    fn function(self, v: f64, first: f64, second: f64) -> Self {
        Self {
            v,
            g: self.g.map(|x| x * first),
            h: std::array::from_fn(|i| {
                std::array::from_fn(|j| first * self.h[i][j] + second * self.g[i] * self.g[j])
            }),
        }
    }
    fn sin(self) -> Self {
        self.function(self.v.sin(), self.v.cos(), -self.v.sin())
    }
    fn cos(self) -> Self {
        self.function(self.v.cos(), -self.v.sin(), -self.v.cos())
    }
}

impl Add for D {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self {
            v: self.v + rhs.v,
            g: std::array::from_fn(|i| self.g[i] + rhs.g[i]),
            h: std::array::from_fn(|i| std::array::from_fn(|j| self.h[i][j] + rhs.h[i][j])),
        }
    }
}

impl Neg for D {
    type Output = Self;
    fn neg(self) -> Self {
        Self {
            v: -self.v,
            g: self.g.map(|x| -x),
            h: self.h.map(|r| r.map(|x| -x)),
        }
    }
}

impl Sub for D {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        self + (-rhs)
    }
}

#[allow(clippy::suspicious_arithmetic_impl)] // Product rule, including mixed second derivatives.
impl Mul for D {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Self {
            v: self.v * rhs.v,
            g: std::array::from_fn(|i| self.g[i] * rhs.v + self.v * rhs.g[i]),
            h: std::array::from_fn(|i| {
                std::array::from_fn(|j| {
                    self.h[i][j] * rhs.v
                        + self.g[i] * rhs.g[j]
                        + self.g[j] * rhs.g[i]
                        + self.v * rhs.h[i][j]
                })
            }),
        }
    }
}

/// Express a vector in the child frame rotated +angle about a principal axis.
fn child_frame(mut v: [D; 3], axis: usize, angle: D) -> [D; 3] {
    let j = (axis + 1) % 3;
    let k = (axis + 2) % 3;
    let (a, b) = (v[j], v[k]);
    v[j] = angle.cos() * a + angle.sin() * b;
    v[k] = -angle.sin() * a + angle.cos() * b;
    v
}

pub fn random(seed: &mut u64, extent: f64) -> f64 {
    *seed = seed
        .wrapping_mul(6364136223846793005)
        .wrapping_add(1442695040888963407);
    (2.0 * ((*seed >> 11) as f64 / ((1_u64 << 53) as f64)) - 1.0) * extent
}

pub fn close(a: f64, b: f64, abs: f64, rel: f64) {
    assert!(
        (a - b).abs() <= abs + rel * a.abs().max(b.abs()),
        "actual={a:.16e}, expected={b:.16e}, error={:.3e}",
        (a - b).abs()
    );
}

/// omega and alpha in housing axes. Physical time derivative of omega_body is
/// alpha_body: the rotating-frame correction is -omega cross omega = zero.
pub fn energy_reference(
    config: SimulationConfig,
    state: JointState,
    drive: DriveState,
    omega: [f64; 3],
    alpha: [f64; 3],
) -> ([f64; 3], f64) {
    let n = config.preset.joint_count();
    let spin = config.preset.spin_index();
    let axes = match config.preset {
        Preset::TurnIndicator => [0, 1, 0],
        Preset::Attitude => [0, 1, 2],
        Preset::Directional => [2, 0, 1],
    };
    let q: [D; 3] = std::array::from_fn(|i| D::variable(state.angles[i], i));
    let v: [D; 3] = std::array::from_fn(|i| D::variable(state.rates[i], i + 3));
    let time = D::variable(0.0, 6);
    let mut w: [D; 3] =
        std::array::from_fn(|i| D::constant(omega[i]) + time * D::constant(alpha[i]));
    let mut kinetic = D::constant(0.0);
    let mut potential = D::constant(0.0);
    for i in 0..n {
        w[axes[i]] = w[axes[i]] + v[i];
        w = child_frame(w, axes[i], q[i]);
        let inertia = if i == spin {
            let mut rotor = [config.rotor.transverse; 3];
            rotor[axes[i]] = config.rotor.axial;
            rotor
        } else if i == 0 {
            config.gimbal_inertia
        } else {
            config.inner_gimbal_inertia
        };
        for j in 0..3 {
            kinetic = kinetic + D::constant(0.5 * inertia[j]) * w[j] * w[j];
        }
        if i < spin {
            let stiffness = if i == 0 {
                config.spring_stiffness
            } else {
                config.inner_spring_stiffness
            };
            potential = potential + D::constant(0.5 * stiffness) * q[i] * q[i];
        }
    }
    let lagrangian = kinetic - potential;
    let mut system = [[0.0; 4]; 3];
    for (i, row) in system.iter_mut().enumerate().take(n) {
        let force = if i == spin {
            (if drive.mode == DriveMode::Powered {
                (config.motor.gain * (drive.target_spin - state.rates[i]))
                    .clamp(-config.motor.torque_limit, config.motor.torque_limit)
            } else {
                0.0
            }) - config.motor.spin_damping * state.rates[i]
        } else {
            -(if i == 0 {
                config.gimbal_damping
            } else {
                config.inner_gimbal_damping
            }) * state.rates[i]
        };
        row[..n].copy_from_slice(&lagrangian.h[i + 3][3..3 + n]);
        row[n] = force + lagrangian.g[i]
            - (0..n)
                .map(|j| lagrangian.h[i + 3][j] * state.rates[j])
                .sum::<f64>()
            - lagrangian.h[i + 3][6];
    }
    (gaussian(system, n), kinetic.v + potential.v)
}

#[allow(clippy::needless_range_loop)]
fn gaussian(mut a: [[f64; 4]; 3], n: usize) -> [f64; 3] {
    for i in 0..n {
        let pivot = (i..n)
            .max_by(|&j, &k| a[j][i].abs().total_cmp(&a[k][i].abs()))
            .unwrap();
        a.swap(i, pivot);
        assert!(a[i][i].abs() > 1e-20);
        for j in i + 1..n {
            let ratio = a[j][i] / a[i][i];
            for k in i..=n {
                a[j][k] -= ratio * a[i][k];
            }
        }
    }
    let mut solution = [0.0; 3];
    for i in (0..n).rev() {
        solution[i] = (a[i][n] - (i + 1..n).map(|j| a[i][j] * solution[j]).sum::<f64>()) / a[i][i];
    }
    solution
}

/// Rodrigues rotation about [1,2,3]/sqrt(14), independent of production math.
pub fn rotated(v: [f64; 3], angle: f64) -> [f64; 3] {
    let a = [1.0, 2.0, 3.0].map(|x| x / 14.0_f64.sqrt());
    let dot = (0..3).map(|i| a[i] * v[i]).sum::<f64>();
    std::array::from_fn(|i| {
        v[i] * angle.cos()
            + (a[(i + 1) % 3] * v[(i + 2) % 3] - a[(i + 2) % 3] * v[(i + 1) % 3]) * angle.sin()
            + a[i] * dot * (1.0 - angle.cos())
    })
}

pub fn orientation(angle: f64) -> [f64; 4] {
    let s = (angle / 2.0).sin() / 14.0_f64.sqrt();
    [s, 2.0 * s, 3.0 * s, (angle / 2.0).cos()]
}
