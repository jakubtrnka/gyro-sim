//! Small internal operations; public interfaces use plain, owned arrays.

use crate::SimulationError;

pub(crate) type Vec3 = [f64; 3];
pub(crate) type Quat = [f64; 4];
pub(crate) type Mat3 = [[f64; 3]; 3];

pub(crate) const ZERO: Vec3 = [0.0; 3];
pub(crate) const X: Vec3 = [1.0, 0.0, 0.0];
pub(crate) const Y: Vec3 = [0.0, 1.0, 0.0];
pub(crate) const Z: Vec3 = [0.0, 0.0, 1.0];
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

pub(crate) fn conjugate(q: Quat) -> Quat {
    [-q[0], -q[1], -q[2], q[3]]
}

pub(crate) fn norm(v: Vec3) -> f64 {
    v[0].hypot(v[1]).hypot(v[2])
}

pub(crate) fn clip_norm(v: Vec3, limit: f64) -> Vec3 {
    let length = norm(v);
    if length.is_infinite() && v.iter().all(|x| x.is_finite()) {
        let largest = v.iter().map(|x| x.abs()).fold(0.0, f64::max);
        let direction = v.map(|x| x / largest);
        return scale(direction, limit / norm(direction));
    }
    if length > limit {
        scale(v, limit / length)
    } else {
        v
    }
}

/// Canonical unit quaternion; deterministic axis-sign tie break at exactly pi.
pub(crate) fn canonical(q: Quat) -> Result<Quat, SimulationError> {
    let q = normalize_quaternion(q)?;
    let flip =
        q[3] < 0.0 || (q[3] == 0.0 && q[..3].iter().find(|x| **x != 0.0).is_some_and(|x| *x < 0.0));
    Ok(if flip { q.map(|x| -x) } else { q })
}

pub(crate) fn rotation_error(target: Quat, actual: Quat) -> Result<Vec3, SimulationError> {
    let q = canonical(quat_mul(target, conjugate(actual)))?;
    let v = [q[0], q[1], q[2]];
    let n = norm(v);
    Ok(if n < 1e-12 {
        scale(v, 2.0)
    } else {
        scale(v, 2.0 * n.atan2(q[3]) / n)
    })
}

/// Checked Cholesky solve of the active 2x2 or 3x3 block, scaled before factoring.
#[allow(clippy::needless_range_loop)] // Matrix indices make the triangular solves explicit.
pub(crate) fn solve_spd(m: Mat3, rhs: Vec3, n: usize) -> Result<(Vec3, f64), SimulationError> {
    let fail = || SimulationError::numerical("mass matrix is not finite positive definite");
    if !m.iter().flatten().chain(rhs.iter()).all(|v| v.is_finite()) || m[0][0] <= 0.0 {
        return Err(fail());
    }
    let largest = (0..n).map(|i| m[i][i]).fold(0.0, f64::max);
    let scaled = m.map(|r| r.map(|x| x / largest));
    let mut l = [[0.0; 3]; 3];
    for i in 0..n {
        for j in 0..=i {
            let value = scaled[i][j] - (0..j).map(|k| l[i][k] * l[j][k]).sum::<f64>();
            l[i][j] = if i == j {
                if value <= 0.0 || !value.is_finite() {
                    return Err(fail());
                }
                value.sqrt()
            } else {
                value / l[j][j]
            };
        }
    }
    let solve = |b: Vec3| {
        let mut x = ZERO;
        for i in 0..n {
            x[i] = (b[i] - (0..i).map(|j| l[i][j] * x[j]).sum::<f64>()) / l[i][i];
        }
        for i in (0..n).rev() {
            x[i] = (x[i] - (i + 1..n).map(|j| l[j][i] * x[j]).sum::<f64>()) / l[i][i];
        }
        x
    };
    let x = solve(rhs.map(|v| v / largest));
    let mut inverse_row_sums = ZERO;
    for j in 0..n {
        let mut unit = ZERO;
        unit[j] = 1.0;
        let column = solve(unit);
        for i in 0..n {
            inverse_row_sums[i] += column[i].abs();
        }
    }
    let matrix_norm = scaled
        .iter()
        .take(n)
        .map(|r| r.iter().take(n).map(|x| x.abs()).sum::<f64>())
        .fold(0.0, f64::max);
    let condition = matrix_norm * inverse_row_sums.into_iter().fold(0.0, f64::max);
    if !x.iter().chain([condition].iter()).all(|x| x.is_finite()) {
        return Err(SimulationError::numerical(
            "linear solve overflow or ill conditioning",
        ));
    }
    Ok((x, condition))
}

#[cfg(test)]
mod tests {
    #[test]
    fn norm_clipping_preserves_direction_when_unscaled_length_overflows() {
        let clipped = super::clip_norm([1.7e308, -1.7e308, 1.7e308], 2.0);
        assert!((super::norm(clipped) - 2.0).abs() < 1e-14);
        assert_eq!(clipped[0], -clipped[1]);
        assert_eq!(clipped[0], clipped[2]);
    }
}
