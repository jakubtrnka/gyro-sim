# Aircraft gyroscope simulation

Problem definition, physics and implementation approach  
Version 0.1 · 24 October 2026 · Design proposal

## 1. Purpose and scope

Build an interactive three-dimensional simulation of mechanical gyroscopes used in aircraft attitude indicators, directional gyros and turn indicators. Run the physical model in Rust, compiled to WebAssembly, and present it in a web browser.

The user should be able to choose a configuration and rotor speed, rotate the instrument housing, apply a torque to a gimbal, and observe the resulting motion. The simulation should explain rigidity in space, precession, nutation and the effect of mechanical constraints through the same equations of motion.

The principal design decision is to model a rotor and its supporting frames as a small system of connected rigid bodies. Instrument presets change the joint arrangement and applied torques. Their behaviour follows from the dynamics.

The first version concerns mechanical instruments. “Attitude indicator” means a spinning-mass gyro mechanism; an electronic ADI can obtain its attitude information from a separate reference system.

This document defines an educational model. Its numerical parameters are illustrative. Reproducing a particular instrument will additionally require its dimensions, inertias, bearing arrangement, erection mechanism and display linkage.

## 2. The mechanical problem

### Components and freedoms

The system comprises an instrument housing, up to two gimbals, and an axisymmetric rotor. A **revolute joint** permits one relative rotation about its bearing axis. It constrains the remaining relative motions and can transmit reaction forces and torques.

For a two-gimbal instrument, choose the internal coordinates

$$
q=(\alpha,\beta,\psi),
$$

where $\alpha$ is the outer-gimbal angle, $\beta$ the inner-gimbal angle and $\psi$ the rotor angle relative to its immediate supporting frame. For a one-gimbal rate gyro, use $q=(\theta,\psi)$.

These coordinates already enforce the joint geometry. Separate equations for the bearing constraints are therefore unnecessary. Aircraft descriptions commonly count the two gimbal freedoms separately from rotor spin; the mechanical model includes all three rotations.

The housing orientation $R_B(t)$ is prescribed by the user or by an experiment. It is an input rather than a dynamic degree of freedom. The housing actuator can supply torque and energy. A freely moving housing can be added later if reaction motion is of interest.

### Coordinates and units

Use a right-handed aircraft frame $B$: $x$ forward, $y$ right and $z$ down. Initially, the inertial world frame $W$ coincides with $B$. Positive rotations follow the right-hand rule. In the initial model, $W$ is a fixed laboratory reference; Earth rotation and transport are omitted.

Let $R_i$ map vector components from component $i$ to $W$:

$$
\boldsymbol v^W=R_i\boldsymbol v^i.
$$

Rotations in a matrix product act on column vectors from right to left. Joint axes belong to their parent frames and move with those frames. All internal quantities use SI units: radians, seconds, kilograms, metres and newton-metres. Convert degrees and RPM at the interface.

### Initial instrument presets

The following axes specify the model at zero joint angles. The outer axis is expressed in the housing, the inner axis in the outer gimbal, and the spin axis in the rotor-supporting frame.

| Preset | Outer axis | Inner axis | Rotor spin axis | Additional physics |
|---|---|---|---|---|
| Attitude indicator | $x$ | $y$ | $z$ | Erection and stops added after the ideal model |
| Directional gyro | $z$ | $x$ | $y$ | Heading-setting offset; later friction and levelling |
| Turn indicator | $x$ | None | $y$ | Torsion spring and damping about the gimbal axis |

These are explicit representative configurations. The distinction between a vertical attitude gyro, a horizontal directional gyro and a spring-restrained rate gyro follows conventional instrument practice.[1,2] Assembly details vary between designs.

A turn coordinator is a later preset with a canted gimbal arrangement. The slip/skid ball is a separate accelerometer-like mechanism and needs its own model.

### Baseline assumptions

- All components are rigid and balanced about a common, fixed pivot.
- Gimbals have finite, non-zero inertia. The rotor is axisymmetric.
- Bearings are ideal initially; friction, springs and damping are explicit options.
- No structural flexibility, collisions between parts or aerodynamic drag are included initially.
- Gravity has no orientation-dependent potential energy for the balanced components. Erection is a separate mechanism.

With a displaced centre of mass, gravitational torque would be $\boldsymbol r_{\rm cm}\times m\boldsymbol g$. A balanced vertical gyro is therefore not automatically made upright by gravity.

## 3. Physical foundations

### Inertia, energy and angular momentum

For material at position $\boldsymbol r$ relative to a body's centre of mass, define the inertia tensor

$$
I=\int\left[(\boldsymbol r\cdot\boldsymbol r)\mathbf 1-\boldsymbol r\boldsymbol r^T\right]dm.
$$

Because a rotating material element has velocity $\boldsymbol\omega\times\boldsymbol r$, integration gives the rotational kinetic energy and angular momentum:

$$
T=\tfrac12\boldsymbol\omega^T I\boldsymbol\omega,
\qquad
\boldsymbol H=I\boldsymbol\omega.
$$

Angular velocity and inertia must be expressed in the same frame. In the rotor's principal axes, choose

$$
I_R=\operatorname{diag}(I_\perp,I_\perp,I_s),
$$

with the third axis along the spindle. More generally, for a local unit spindle direction $\boldsymbol s_0$,

$$
I_R=I_\perp\mathbf 1+(I_s-I_\perp)\boldsymbol s_0\boldsymbol s_0^T.
$$

Thus a rotor spinning about local $y$ uses $\operatorname{diag}(I_\perp,I_s,I_\perp)$ in those axes.

The governing angular-momentum law, about the fixed pivot or a body's centre of mass, is

$$
\left.\frac{d\boldsymbol H}{dt}\right|_W=\boldsymbol\tau.
$$

In body-fixed coordinates, it becomes Euler's rigid-body equation:

$$
I\dot{\boldsymbol\omega}+\boldsymbol\omega\times(I\boldsymbol\omega)=\boldsymbol\tau.
$$

The cross product accounts for the rotating coordinate frame.[3] A simulation that uses only $I\dot{\boldsymbol\omega}=\boldsymbol\tau$ in body coordinates generally misses gyroscopic coupling.

### Rigidity, precession and nutation

For rapid spin, rotor angular momentum is approximately

$$
\boldsymbol H_R\simeq h\boldsymbol s,
\qquad h=I_s\Omega_s,
$$

where $\boldsymbol s$ is the unit spindle direction and $h$ is signed axial momentum. With negligible net torque, angular momentum remains constant in inertial space. At high spin this makes the spindle approximately maintain its direction.

If its direction changes slowly with precession angular velocity $\boldsymbol\Omega_p$, then

$$
\boldsymbol\tau\simeq\boldsymbol\Omega_p\times\boldsymbol H_R.
$$

For perpendicular precession, $|\Omega_p|\simeq|\tau|/|h|$. This is a high-spin approximation, not the complete transient dynamics.[4] Nutation is the additional oscillation that can accompany changes in spindle direction. Finite inertia and the full equations determine it.

Thus the “90°” description of precession is a consequence of a vector cross product. It is not a delayed force that must be inserted into the program.

## 4. A short introduction to Lagrangian mechanics

### Why use it?

Newton–Euler equations could be written for every component, together with all bearing reactions. Lagrangian mechanics expresses the same mechanics in coordinates that already satisfy the constraints. Ideal joint reactions then disappear from the equations for the permitted motions.[5]

For our mechanism, the natural variables are joint angles. The method is:

1. Describe the configuration using independent coordinates $q_j$.
2. Express kinetic energy $T(q,\dot q,t)$ and potential energy $V(q,t)$.
3. Form the **Lagrangian** $L=T-V$.
4. Apply one equation for each coordinate:

$$
\boxed{
\frac{d}{dt}\frac{\partial L}{\partial\dot q_j}
-\frac{\partial L}{\partial q_j}=Q_j.
}
$$

Here $Q_j$ represents applied effects not already included in $V$, such as a motor or damper. It is defined by virtual work,

$$
\delta W=\sum_j Q_j\,\delta q_j.
$$

For an ordinary revolute-joint angle, $Q_j$ is the corresponding joint torque. Frictionless constraints do no virtual work for allowed variations at a fixed time. A moving support can nevertheless do actual work.

### Where does the equation come from?

For the conservative system, define the action

$$
S[q]=\int_{t_0}^{t_1}L(q,\dot q,t)\,dt.
$$

The actual trajectory makes the action stationary under small admissible changes with fixed endpoints. Integrating the term involving $\delta\dot q_j$ by parts gives

$$
\delta S=
\int_{t_0}^{t_1}\sum_j
\left[\frac{\partial L}{\partial q_j}
-\frac{d}{dt}\frac{\partial L}{\partial\dot q_j}\right]\delta q_j\,dt.
$$

Since the independent variations are arbitrary, each coefficient must vanish. Including applied virtual work through $\delta S+\int\sum_jQ_j\delta q_j\,dt=0$ gives the forced equation above. “Stationary” need not mean a minimum; it does not mean that the system minimises its instantaneous energy.

### One simple example

For a frame with inertia $J$, a torsion spring $k$ and angular displacement $\theta$,

$$
T=\tfrac12J\dot\theta^2,
\qquad V=\tfrac12k\theta^2.
$$

Add an applied torque $\tau$ and viscous damping $-c\dot\theta$. Lagrange's equation yields

$$
J\ddot\theta+c\dot\theta+k\theta=\tau.
$$

The turn indicator will have this form with a gyroscopic driving term. In a gyro, the extra coupling arises because the component angular velocities depend on several joint rates at once.

## 5. Constructing the complete model

### Orientations and angular velocities

For the attitude-gyro configuration, the component-to-world rotations are

$$
R_O=R_B R_x(\alpha),\qquad
R_G=R_O R_y(\beta),\qquad
R_R=R_G R_z(\psi).
$$

For the directional gyro, substitute the axis sequence $z,x,y$. For the turn indicator, use $R_G=R_B R_x(\theta)$ and $R_R=R_G R_y(\psi)$.

In a serial chain, each component's world angular velocity is

$$
\boldsymbol\omega_i^W=
\boldsymbol\omega_B^W+
\sum_{j\in\operatorname{ancestors}(i)}\dot q_j\boldsymbol a_j^W,
$$

where $\boldsymbol a_j^W$ is the current unit axis of joint $j$. The sum includes the component's own joint. Transform to component coordinates with $\boldsymbol\omega_i^i=R_i^T\boldsymbol\omega_i^W$.

This is why three angle derivatives cannot generally be treated as three orthogonal angular-velocity components.

### Energies and forces

For the balanced, concentric mechanism,

$$
T=\frac12\sum_i(\boldsymbol\omega_i^i)^T I_i\boldsymbol\omega_i^i.
$$

Add $\tfrac12m_i|\boldsymbol v_{{\rm cm},i}|^2$ if later versions introduce offset centres of mass. Springs contribute $\tfrac12k_j(q_j-q_{0j})^2$ to $V$; viscous joint damping contributes $-c_j\dot q_j$ to $Q_j$. Do not include the same spring or torque twice.

The unknown trajectory is determined by the selected geometry, inertias, torques, housing motion and initial values $(q_0,\dot q_0)$.

### Equations suitable for computation

Let $v=\dot q$, $p=\partial L/\partial v$ and

$$
M=\frac{\partial^2 L}{\partial v\,\partial v^T}.
$$

Expanding the total derivative gives

$$
\boxed{
M\ddot q=
Q+\frac{\partial L}{\partial q}
-\frac{\partial p}{\partial q}\,v
-\left.\frac{\partial p}{\partial t}\right|_{q,v}.
}
$$

The last term includes the prescribed housing motion. Its angular velocity and angular acceleration must be consistent with its orientation. Evaluate all of them at the solver's current stage time.

Derive these small systems analytically or by automatic differentiation during development. Production code should evaluate explicit expressions and solve the linear system for $\ddot q$ without explicitly forming $M^{-1}$.

## 6. Worked example: two gimbals and a spinning rotor

This section derives an exact special case of the proposed model. It also provides a reference against which to check the general implementation.

Fix the housing, use the $x,y,z$ joint sequence, and omit potential energy. Let the outer gimbal's inertia about its permitted $x$ axis be $A$, and the inner gimbal's principal inertia tensor be $\operatorname{diag}(B_1,B_2,B_3)$.

The inner gimbal's angular velocity, expressed in its own axes, is

$$
\boldsymbol\omega_G^G=
\begin{pmatrix}
\dot\alpha\cos\beta\\
\dot\beta\\
\dot\alpha\sin\beta
\end{pmatrix}.
$$

The rotor adds $\dot\psi$ along its spindle. Axisymmetry makes its energy independent of the instantaneous spin phase:

$$
T_R=\tfrac12I_\perp
\left(\dot\beta^2+\dot\alpha^2\cos^2\beta\right)
+\tfrac12I_s\left(\dot\psi+\dot\alpha\sin\beta\right)^2.
$$

Define

$$
D(\beta)=A+(B_1+I_\perp)\cos^2\beta+B_3\sin^2\beta,
\qquad J_\beta=B_2+I_\perp.
$$

Adding the gimbals gives

$$
T=\tfrac12D(\beta)\dot\alpha^2
+\tfrac12J_\beta\dot\beta^2
+\tfrac12I_s\left(\dot\psi+\dot\alpha\sin\beta\right)^2.
$$

Because $T$ does not depend on $\psi$, that angle is a **cyclic coordinate**. Its conjugate momentum is

$$
h=\frac{\partial L}{\partial\dot\psi}
=I_s\left(\dot\psi+\dot\alpha\sin\beta\right),
\qquad \dot h=Q_\psi.
$$

Therefore, with no axial driving torque or spin-bearing friction, $h$ is constant. Notice that constant $h$ does not require constant spin speed $\dot\psi$ relative to the gimbal.

Direct application of Lagrange's equations yields

$$
D\ddot\alpha+D'\dot\alpha\dot\beta
+h\cos\beta\,\dot\beta+\dot h\sin\beta=Q_\alpha,
$$

$$
J_\beta\ddot\beta-\tfrac12D'\dot\alpha^2
-h\cos\beta\,\dot\alpha=Q_\beta,
$$

where

$$
D'=2(B_3-B_1-I_\perp)\sin\beta\cos\beta.
$$

These equations retain finite gimbal inertia and arbitrary spin speed under the stated assumptions. Motor reaction effects appear through $\dot h$ and the coupled motion.

### Seeing the gyroscopic coupling

Near $\beta=0$, with constant $h$ and small gimbal rates, the leading equations are

$$
J_\alpha\ddot\alpha+h\dot\beta=Q_\alpha,
\qquad
J_\beta\ddot\beta-h\dot\alpha=Q_\beta,
$$

where $J_\alpha=A+B_1+I_\perp$.

In a slowly varying response, a torque about $\alpha$ produces approximately $\dot\beta=Q_\alpha/h$. Reversing spin reverses this coupling. The complete equations also contain the transient accelerations and remain usable at zero spin; the divided-by-$h$ approximation does not.

For the unforced linearised system, the joint rates oscillate with characteristic angular frequency

$$
\omega_n=\frac{|h|}{\sqrt{J_\alpha J_\beta}}.
$$

This gives a useful numerical check and explains why a rapidly spinning gyro can require small integration steps even though its visible precession is slow.

### Removing the fast spin phase correctly

For constant $h$, it is possible to solve only for the two gimbal angles. Eliminating a cyclic velocity requires the **Routhian**

$$
\mathcal R=L-h\dot\psi
=\tfrac12D\dot\alpha^2+\tfrac12J_\beta\dot\beta^2
+h\dot\alpha\sin\beta-\frac{h^2}{2I_s}.
$$

Apply the Lagrange equations to $\mathcal R$ with $h$ held fixed. Simply substituting $\dot\psi=h/I_s-\dot\alpha\sin\beta$ into $L$ and treating that expression as a new Lagrangian would lose the gyroscopic term.

For motors and spin-down, retain the full equations or evolve $h$ consistently through $\dot h=Q_\psi$. Spin phase can then be reconstructed for rendering.

## 7. Turn-indicator behaviour

Use a rotor spinning initially about aircraft $+y$, a gimbal freedom $\theta$ about $+x$, and an imposed housing yaw rate $\Omega$ about $+z$. Add a spring $V=\tfrac12k\theta^2$ and damping $-c\dot\theta$.

In the high-spin, small-deflection limit, neglecting terms quadratic in the imposed yaw rate, the model reduces to

$$
\boxed{J_{\rm eff}\ddot\theta+c\dot\theta+k\theta=h\Omega.}
$$

The sign follows the chosen axes and spin direction. At zero deflection the rotor requires a torque proportional to $-h\Omega$ about $x$; the corresponding reaction on its supporting gimbal drives positive $\theta$.

For a settled response,

$$
\theta_{\rm eq}\simeq\frac{h}{k}\Omega.
$$

The spring turns a rate-dependent gyroscopic torque into a measurable angle. Damping determines settling, and reduced rotor speed reduces sensitivity. A calibration mark at $\theta_{\rm ref}$ for a nominal rate $\Omega_{\rm ref}$ requires $k=h_{\rm nominal}\Omega_{\rm ref}/\theta_{\rm ref}$ in this approximation. A two-minute turn corresponds to $\Omega_{\rm ref}=3^\circ/{\rm s}$.

For an illustrative thin disc of mass $0.10\,\rm kg$ and radius $0.025\,\rm m$ at $12{,}000$ RPM,

$$
I_s=\tfrac12mr^2=3.125\times10^{-5}\,\rm kg\,m^2,
\qquad h\simeq0.03927\,\rm N\,m\,s.
$$

At $3^\circ/{\rm s}$, $h\Omega\simeq2.06\times10^{-3}\,\rm N\,m$. These are example scales, not specifications for a particular instrument.

A conventional rate gyro senses a component of housing angular velocity. In arbitrary bank and pitch, body-axis yaw rate is not generally the same as the time derivative of heading. The displayed quantity must follow the actual mechanism rather than an assumed heading-rate input.

## 8. Meaning of the interactive controls

### Moving the housing

Dragging a housing rotation handle supplies a target orientation. Generate a smooth trajectory $R_B(t)$, with consistent angular velocity and angular acceleration. A sudden pose jump has undefined impulsive motion and should be reserved for resetting the experiment.

Use quaternions for the housing orientation. Specify that the quaternion maps $B$ to $W$; if using body-resolved angular velocity and Hamilton multiplication, its derivative is $\dot u=\tfrac12u\otimes(0,\boldsymbol\omega_B^B)$. Quaternion multiplication order and component order must be explicit at the Rust–JavaScript boundary.

### Pushing a gimbal

Dragging a gimbal should apply a bounded virtual spring–damper torque,

$$
Q_{\rm mouse}=
\operatorname{clip}\!\left[k_m(q_{\rm target}-q)-c_m\dot q,
-\tau_{\max},\tau_{\max}\right].
$$

The selected angle is a mechanical joint coordinate. Releasing the mouse removes this torque. Directly overwriting the angle would prescribe motion and suppress the dynamic response being investigated.

### Choosing rotor speed

Provide two distinct controls: an initial RPM used when resetting, and a motor target RPM used during a run. For example, a torque-limited speed controller can use

$$
Q_\psi=\operatorname{clip}
\left[K_m(\dot\psi_{\rm target}-\dot\psi),-\tau_{m,\max},\tau_{m,\max}\right]
-c_s\dot\psi.
$$

Here the target refers to rotation relative to the supporting frame. The motor and bearing apply equal and opposite torques to rotor and support; treating them as joint torques accounts for this through virtual work.

## 9. Numerical solution

Use a dedicated solver for the small system of joint coordinates. At each evaluation, obtain the prescribed housing motion, compute orientations and angular velocities, assemble the dynamics, and solve for accelerations. Integrate the first-order state $\dot q=v$, $\dot v=M^{-1}b$ using a linear solve in the implementation.

Begin with fourth-order Runge–Kutta and a fixed simulation step chosen by convergence tests. Use `f64` for physics. Check symmetry and positive definiteness of the mass matrix for finite, physically valid component inertias.

The rendering rate and physics step are independent. Advance physics in substeps and interpolate component poses for display. Slow motion scales simulated time while preserving the physical parameters.

For an axisymmetric rotor, the equations do not depend on its spin phase. This avoids resolving individual rotor revolutions merely to update the spindle direction. Fast nutation and stiff springs can still restrict the step size. If those modes become too expensive, evaluate an implicit or structure-preserving method, or offer a separately labelled high-spin approximation.

Mechanical stops require an explicit contact or compliant-stop model. Gimbal lock is a physical loss of an independent pointing direction when axes align. Quaternions make orientation representation robust but do not remove the mechanical condition. Keep finite gimbal inertias and report poor conditioning rather than hiding it with arbitrary corrections.

## 10. Software and presentation

| Part | Responsibility |
|---|---|
| `gyro-core`, pure Rust | Geometry, inertias, joint torques, equations, integration and diagnostics; usable in native tests |
| `gyro-wasm` | Small `wasm-bindgen` interface for configuration, commands, stepping and state snapshots |
| TypeScript and Three.js | Scene, meshes, camera, picking, rotation handles and controls |
| HTML/SVG | Instrument dial, numeric values and time-series plots |

`wasm-bindgen` supports the Rust–JavaScript interface, and Three.js provides browser rendering through WebGL 2.[6,7] This is the proposed implementation split; exact dependency versions should be pinned when implementation begins.

Export component poses, joint angles and rates, signed rotor momentum, applied torques and energy diagnostics in one snapshot. Store experiments as configuration, initial state and time-stamped commands. Given the same step size and inputs, native and WASM results should agree within stated floating-point tolerances.

The main view should show the exposed mechanism with labelled axes and an adjacent instrument dial. Offer world-fixed and housing-fixed cameras, pause and single-step, rotor-speed and spin-direction controls, repeatable manoeuvres, and optional torque/angular-momentum vectors.

Apply one fixed, orientation-preserving conversion between the aircraft coordinates and the renderer's scene coordinates. Camera conventions must not change the signs used in the physics.

The dial must read the simulated mechanism. For the ideal attitude display, use the simulated spindle direction expressed in housing coordinates. For a DG during a level yaw experiment, the indicated heading is the setting offset minus the outer-gimbal angle. Extend the readout using the selected mechanism's geometry for general attitudes.

At high RPM, render a blurred rotor and optionally a clearly labelled slowed spin marker. Keep this visual convention separate from simulation time.

## 11. Validation and implementation stages

### Checks that establish physical correctness

| Check | Expected result |
|---|---|
| Fixed housing, ideal joints, no applied torques or springs | Total kinetic energy, axial momentum $h$ and the cyclic-coordinate momentum $p_\alpha$ remain constant |
| General energy-based solver versus Section 6 | Matching accelerations for arbitrary nonsingular states and specified joint torques |
| Small-motion unforced two-gimbal case | Joint-rate oscillation frequency approaches $|h|/\sqrt{J_\alpha J_\beta}$ |
| Slow forced precession at high spin | Rate approaches the predicted torque-to-momentum ratio, with any transient resolved |
| Turn indicator at steady small yaw rate | Deflection approaches $h\Omega/k$ |
| Reverse initial spin | Sign of gyroscopic coupling reverses |
| Zero initial spin | Ordinary constrained rigid-body dynamics; no division by spin speed |
| Halve the integration step | Differences decrease consistently with the solver's order in smooth, resolved cases |
| Drive the housing or motor | Energy change agrees with actuator work and losses, allowing for numerical error |

A fixed housing can exert external constraint torques. Do not require conservation of the assembly's entire angular-momentum vector merely because its bearings are frictionless. For a separately simulated isolated torque-free rotor, conservation of that full vector is an appropriate check.

For constant-$h$ tests, also inspect energy after subtracting the constant spin term $h^2/(2I_s)$. Its large magnitude could otherwise mask errors in the much smaller gimbal-motion energy. Record absolute errors and define numerical acceptance tolerances before treating a model as validated.

For this draft, the explicit fixed-housing equations were compared with a second-order automatic differentiation of kinetic energy assembled independently from world-space angular velocities. Two hundred test states, including non-zero spin torques, agreed to a scaled acceleration discrepancy below $8\times10^{-13}$. A smooth unforced example gave an RK4 step-halving difference ratio of $16.04$, consistent with fourth-order convergence, while conserving dynamic energy and $p_\alpha$ to decreasing numerical error. These checks support the worked derivation; the proposed moving-housing simulator remains to be implemented and validated.

### Development sequence

1. Implement the two-gimbal fixed-housing equations and their validation cases in native Rust.
2. Add prescribed housing motion and verify it against the fixed-housing special case and repeatable manoeuvres.
3. Add the one-gimbal spring/damper configuration and check the rate-gyro limit.
4. Expose the solver through WASM and build the interactive mechanism view and instrument readouts.
5. Add instrument-specific effects: friction, motor spin-up/down, erection, caging and mechanical stops.

Later, Earth rotation, transport and aircraft translation can extend the reference-frame model. A physical erection system must respond to its actual gravity/acceleration sensing mechanism; using the true world vertical as a perfect reference would conceal acceleration errors. Those extensions need separate specifications and validation cases.

## References

The worked equations in Sections 5–7 are derived for the geometry defined here. The following primary sources provide the underlying mechanics, conventional instrument arrangements and software interfaces.

1. FAA, *Instrument Flying Handbook*, FAA-H-8083-15B, Chapter 5, especially pp. 5-19–5-21: gyroscopic instrument arrangements and rate-gyro restraint. [Handbook PDF](https://www.faa.gov/sites/faa.gov/files/pilots/FAA-H-8083-15B.pdf).
2. UK Ministry of Defence, *AP3456, Volume 5: Flight Instruments*, Chapters 11 and 21: gyroscope principles, gimbal arrangements and attitude-indicator erection. [Manual PDF](https://assets.publishing.service.gov.uk/media/5e18affaed915d3b1410a57f/Volume_05_Flight_Instruments.pdf).
3. MIT OpenCourseWare, 16.07 Dynamics, Lecture 28, *3D Rigid Body Dynamics: Equations of Motion; Euler's Equations*. [Lecture PDF](https://ocw.mit.edu/courses/16-07-dynamics-fall-2009/5e1d8699338146e5127080b880b906d6_MIT16_07F09_Lec28.pdf).
4. MIT OpenCourseWare, 16.07 Dynamics, Lecture 30, *3D Rigid Body Dynamics: Tops and Gyroscopes*. [Lecture PDF](https://ocw.mit.edu/courses/16-07-dynamics-fall-2009/99f271d601bb3d9d04e8b8ce0c73cb7a_MIT16_07F09_Lec30.pdf).
5. MIT OpenCourseWare, 16.07 Dynamics, Lecture 20, *Energy Methods: Lagrange's Equations*. [Lecture PDF](https://ocw.mit.edu/courses/16-07-dynamics-fall-2009/b39e882f1524a0f6a98553ee33ea6f35_MIT16_07F09_Lec20.pdf).
6. *The wasm-bindgen Guide*. [Official documentation](https://wasm-bindgen.github.io/wasm-bindgen/).
7. Three.js, *WebGLRenderer*. [Official documentation](https://threejs.org/docs/pages/WebGLRenderer.html).
