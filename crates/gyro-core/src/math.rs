//! Small internal operations; public interfaces use plain, owned arrays.

use crate::SimulationError;

pub(crate) type Vec3 = [f64; 3];
pub(crate) type Quat = [f64; 4];
pub(crate) type Mat3 = [[f64; 3]; 3];

pub(crate) const ZERO: Vec3 = [0.0; 3];
pub(crate) const X: Vec3 = [1.0, 0.0, 0.0];
pub(crate) const Y: Vec3 = [0.0, 1.0, 0.0];
pub(crate) const IDENTITY: Quat = [0.0, 0.0, 0.0, 1.0];

pub(crate) fn add(a: Vec3, b: Vec3) -> Vec3 {
    std::array::from_fn(|i| a[i] + b[i])
}

pub(crate) fn scale(a: Vec3, s: f64) -> Vec3 {
    a.map(|x| x * s)
}

pub(crate) fn dot(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub(crate) fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

pub(crate) fn mat_vec(m: Mat3, v: Vec3) -> Vec3 {
    m.map(|row| dot(row, v))
}

pub(crate) fn normalize_quaternion(q: Quat) -> Result<Quat, SimulationError> {
    if !q.iter().all(|x| x.is_finite()) {
        return Err(SimulationError::InvalidInput("quaternion must be finite"));
    }
    let largest = q.iter().map(|x| x.abs()).fold(0.0, f64::max);
    if largest < 1e-12 {
        return Err(SimulationError::InvalidInput(
            "quaternion norm is effectively zero",
        ));
    }
    // Scaling first also handles finite inputs whose unscaled norm would overflow.
    let scaled = q.map(|x| x / largest);
    let norm = scaled.iter().map(|x| x * x).sum::<f64>().sqrt();
    Ok(scaled.map(|x| x / norm))
}

pub(crate) fn quat_mul(a: Quat, b: Quat) -> Quat {
    let av = [a[0], a[1], a[2]];
    let bv = [b[0], b[1], b[2]];
    let v = add(add(scale(bv, a[3]), scale(av, b[3])), cross(av, bv));
    [v[0], v[1], v[2], a[3] * b[3] - dot(av, bv)]
}

pub(crate) fn axis_rotation(axis: Vec3, angle: f64) -> Quat {
    let (s, c) = (0.5 * angle).sin_cos();
    [axis[0] * s, axis[1] * s, axis[2] * s, c]
}

pub(crate) fn rotation_matrix(q: Quat) -> Mat3 {
    let [x, y, z, w] = q;
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

pub(crate) fn rotate_diagonal(r: Mat3, inertia: Vec3) -> Mat3 {
    std::array::from_fn(|i| {
        std::array::from_fn(|j| (0..3).map(|k| r[i][k] * inertia[k] * r[j][k]).sum())
    })
}

pub(crate) fn axial_inertia(spindle: Vec3, axial: f64, transverse: f64) -> Mat3 {
    std::array::from_fn(|i| {
        std::array::from_fn(|j| {
            (if i == j { transverse } else { 0.0 }) + (axial - transverse) * spindle[i] * spindle[j]
        })
    })
}

/// Checked Cholesky solve, with a scale-independent infinity-norm condition estimate.
pub(crate) fn solve_spd(
    m: [[f64; 2]; 2],
    rhs: [f64; 2],
) -> Result<([f64; 2], f64), SimulationError> {
    let fail = || SimulationError::numerical("mass matrix is not finite positive definite");
    if !m.iter().flatten().chain(rhs.iter()).all(|v| v.is_finite()) || m[0][0] <= 0.0 {
        return Err(fail());
    }
    let l00 = m[0][0].sqrt();
    let l10 = m[1][0] / l00;
    let remaining = m[1][1] - l10 * l10;
    if remaining <= 0.0 || !remaining.is_finite() {
        return Err(fail());
    }
    let l11 = remaining.sqrt();
    let y0 = rhs[0] / l00;
    let y1 = (rhs[1] - l10 * y0) / l11;
    let x1 = y1 / l11;
    let x0 = (y0 - l10 * x1) / l00;
    let largest = m[0][0].max(m[1][1]);
    let (a, b, c) = (m[0][0] / largest, m[0][1] / largest, m[1][1] / largest);
    let norm = (a + b.abs()).max(c + b.abs());
    let condition = norm * norm / (a * c - b * b);
    if ![x0, x1, condition].iter().all(|x| x.is_finite()) {
        return Err(SimulationError::numerical(
            "linear solve overflow or ill conditioning",
        ));
    }
    Ok(([x0, x1], condition))
}
