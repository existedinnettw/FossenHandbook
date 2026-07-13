# Chapter 3 - Rigid-Body Kinetics

Kinetics is the force side of dynamics. Chapter 2 gave the geometry of motion,

$$
\dot{\eta} = J(\eta)\nu,
$$

while Chapter 3 derives the rigid-body equations that determine how the BODY-fixed velocity $\nu$ changes when forces and moments act on the craft.

This note is organized by concept instead of slide order.

## Overview

- **Newton-Euler equations** combine translational momentum balance and angular momentum balance.
- **NED is treated as inertial** for ordinary marine craft modeling: $\{n\} \approx \{i\}$.
- **BODY coordinates rotate**, so Coriolis and centripetal terms appear when inertial equations are expressed in $\{b\}$.
- **CG equations are physically natural**, but **CO equations are control-natural** because the coordinate origin can be chosen for the GNC objective.
- **Rigid-body dynamics** are written compactly as

$$
M_{RB}\dot{\nu} + C_{RB}(\nu)\nu = \tau_{RB}.
$$

where:

- $M_{RB}$: rigid-body system inertia matrix.
- $C_{RB}(\nu)$: rigid-body Coriolis and centripetal matrix.
- $\nu = [u, v, w, p, q, r]^T$: generalized BODY-fixed velocity.
- $\tau_{RB} = [X, Y, Z, K, M, N]^T$: generalized force and moment vector.

## Foundations

### Frames and Reference Points

Chapter 3 uses two body-fixed reference points:

- **CG**, subscript $g$: center of gravity.
- **CO**, subscript $b$: coordinate origin of the BODY frame $\{b\}$.

For control and simulation, the CO is usually preferred. The CG can move when payload, ballast, or fuel distribution changes, while the CO can be fixed to a useful control point:

- a stationkeeping point,
- a path-following point,
- a sensor or actuator reference point,
- a nominal point on the hull.

Let

$$
r_g^b =
\begin{bmatrix}
x_g & y_g & z_g
\end{bmatrix}^T
$$

be the vector from the CO to the CG, expressed in BODY coordinates.

If $r_g^b = 0$, the CO and CG coincide and many coupling terms vanish.

### Time Differentiation in a Rotating Frame

For any vector $a$, inertial differentiation and BODY differentiation are related by

$$
\left(\frac{d a}{dt}\right)_n
=
\left(\frac{d a}{dt}\right)_b
+ \omega_{nb}^b \times a.
$$

Using the skew-symmetric operator $S(\cdot)$:

$$
\left(\frac{d a}{dt}\right)_n
=
\dot{a}^b + S(\omega_{nb}^b)a^b.
$$

This is the source of the extra velocity-product terms in BODY-fixed dynamics.

### Velocity Partition

It is useful to split the 6-DOF velocity into linear and angular parts:

$$
\nu =
\begin{bmatrix}
\nu_1 \\
\nu_2
\end{bmatrix},
\qquad
\nu_1 =
\begin{bmatrix}
u \\ v \\ w
\end{bmatrix},
\qquad
\nu_2 =
\begin{bmatrix}
p \\ q \\ r
\end{bmatrix}.
$$

In this chapter:

$$
\nu_1 = v_{nb}^b,
\qquad
\nu_2 = \omega_{nb}^b.
$$

## Newton-Euler Equations About the CG

### Assumptions

The chapter starts from two standard marine-craft assumptions:

1. The vessel is rigid.
2. The local NED frame is approximately inertial, $\{n\} \approx \{i\}$.

The second assumption ignores small inertial effects due to Earth rotation. Earth rotates at

$$
\omega_{ie} = 7.2921 \times 10^{-5}\ \mathrm{rad/s},
$$

which is usually small compared with marine hydrodynamic effects.

### Translational Motion

Newton's second law at the CG is

$$
m a_g = f_g.
$$

Expressed in BODY coordinates:

$$
m(\dot{v}_g^b + \omega_{nb}^b \times v_g^b) = f_g^b.
$$

Using $\nu_1 = v_g^b$ and $\nu_2 = \omega_{nb}^b$ when the origin is at the CG:

$$
m(\dot{\nu}_1 + S(\nu_2)\nu_1) = f_g^b.
$$

### Rotational Motion

Euler's rotational equation about the CG is

$$
I_g^b \dot{\omega}_{nb}^b
+ \omega_{nb}^b \times (I_g^b\omega_{nb}^b)
= m_g^b,
$$

or

$$
I_g^b\dot{\nu}_2 + S(\nu_2)I_g^b\nu_2 = m_g^b.
$$

The inertia tensor about the CG is

$$
I_g^b =
\begin{bmatrix}
I_x & -I_{xy} & -I_{xz} \\
-I_{yx} & I_y & -I_{yz} \\
-I_{zx} & -I_{zy} & I_z
\end{bmatrix}.
$$

The diagonal terms are moments of inertia. The off-diagonal terms are products of inertia.

### Matrix Form at the CG

With the coordinate origin at the CG:

$$
M_{RB}^{CG}\dot{\nu} + C_{RB}^{CG}(\nu)\nu = \tau_{RB}^{CG}
$$

where

$$
M_{RB}^{CG} =
\begin{bmatrix}
mI_3 & 0_{3 \times 3} \\
0_{3 \times 3} & I_g^b
\end{bmatrix},
$$

and

$$
C_{RB}^{CG}(\nu) =
\begin{bmatrix}
mS(\nu_2) & 0_{3 \times 3} \\
0_{3 \times 3} & -S(I_g^b\nu_2)
\end{bmatrix}.
$$

Then

$$
C_{RB}^{CG}(\nu)\nu =
\begin{bmatrix}
mS(\nu_2)\nu_1 \\
-S(I_g^b\nu_2)\nu_2
\end{bmatrix}
=
\begin{bmatrix}
m\omega \times v_g \\
\omega \times I_g\omega
\end{bmatrix},
$$

because $-S(I_g\omega)\omega = \omega \times I_g\omega$.

## Transforming from CG to CO

### Why Use the CO?

The CG is excellent for deriving equations, but the control system often cares about another point. Fossen's marine craft model therefore represents rigid-body equations about the BODY-fixed coordinate origin CO.

The same physical rigid body can be represented at the CO by transforming:

- velocity,
- force/moment,
- mass matrix,
- Coriolis/centripetal matrix.

### System Transformation Matrix

The CG-to-CO transformation uses the $H$ matrix:

$$
H(r_g^b) =
\begin{bmatrix}
I_3 & -S(r_g^b) \\
0_{3 \times 3} & I_3
\end{bmatrix}.
$$

The CG velocity is related to the CO velocity by

$$
\nu_g = H(r_g^b)\nu_b.
$$

This says:

$$
v_g^b = v_b^b + \omega_{nb}^b \times r_g^b,
\qquad
\omega_g^b = \omega_b^b.
$$

### Parallel-Axis Theorem

The inertia tensor about the CO is

$$
I_b^b = I_g^b - mS(r_g^b)S(r_g^b).
$$

This is the compact matrix form of the parallel-axis theorem.

For $r_g^b = [x_g, y_g, z_g]^T$:

$$
-S(r_g^b)S(r_g^b)
=
\|r_g^b\|^2I_3 - r_g^b(r_g^b)^T.
$$

So moving the reference point away from the CG increases the apparent rotational inertia according to the distance from the CG.

## Newton-Euler Equations About the CO

### Rigid-Body Mass Matrix

The 6-DOF rigid-body inertia matrix about the CO is

$$
M_{RB} =
\begin{bmatrix}
mI_3 & -mS(r_g^b) \\
mS(r_g^b) & I_b^b
\end{bmatrix}.
$$

Important properties:

- $M_{RB} = M_{RB}^T$.
- $M_{RB}$ is positive definite for a physical rigid body.
- The off-diagonal blocks encode translation-rotation coupling caused by $r_g^b \neq 0$.

If the CO equals the CG, $r_g^b = 0$, then

$$
M_{RB} =
\begin{bmatrix}
mI_3 & 0 \\
0 & I_g^b
\end{bmatrix}.
$$

### Translational Motion About the CO

The velocity of the CG is

$$
v_g^b = \nu_1 + S(\nu_2)r_g^b.
$$

The translational equation about the CO becomes

$$
m\left[
\dot{\nu}_1
- S(r_g^b)\dot{\nu}_2
+ S(\nu_2)\nu_1
- S(\nu_2)S(r_g^b)\nu_2
\right]
= f^b.
$$

Equivalent vector form:

$$
m\left[
\dot{v}
+ \dot{\omega}\times r_g
+ \omega\times v
+ \omega\times(\omega\times r_g)
\right]
= f.
$$

The new terms come from the offset between CO and CG:

- $\dot{\omega}\times r_g$: tangential acceleration of the CG due to angular acceleration.
- $\omega\times(\omega\times r_g)$: centripetal acceleration of the CG due to rotation.

### Rotational Motion About the CO

The rotational equation about the CO can be written compactly as the lower three rows of

$$
M_{RB}\dot{\nu} + C_{RB}(\nu)\nu = \tau_{RB}.
$$

In vector form, the CO moment balance includes:

- angular acceleration through $I_b^b\dot{\omega}$,
- gyroscopic coupling through $\omega \times I_b^b\omega$,
- force/moment coupling from the offset $r_g^b$.

The important modeling lesson is that the moment equation about CO is not simply the CG moment equation with $I_g$ renamed. The offset introduces translation-rotation coupling in both $M_{RB}$ and $C_{RB}$.

## Rigid-Body Matrix-Vector Model

### Main Equation

The rigid-body kinetics of a marine craft are written as

$$
M_{RB}\dot{\nu} + C_{RB}(\nu)\nu = \tau_{RB}.
$$

Combined with Chapter 2 kinematics:

$$
\dot{\eta} = J(\eta)\nu,
$$

the rigid-body state model is

$$
\begin{aligned}
\dot{\eta} &= J(\eta)\nu, \\
\dot{\nu} &= M_{RB}^{-1}\left[\tau_{RB} - C_{RB}(\nu)\nu\right].
\end{aligned}
$$

Hydrodynamics, hydrostatics, control forces, and environmental loads are added in later chapters.

The full marine craft model will become

$$
M\dot{\nu} + C(\nu)\nu + D(\nu)\nu + g(\eta) = \tau + \tau_{env},
$$

where Chapter 3 contributes the rigid-body parts $M_{RB}$ and $C_{RB}$.

### Coriolis and Centripetal Matrix from a System Inertia Matrix

Let a constant 6-by-6 inertia matrix be partitioned as

$$
M =
\begin{bmatrix}
M_{11} & M_{12} \\
M_{21} & M_{22}
\end{bmatrix},
\qquad
M_{21} = M_{12}^T.
$$

Define

$$
a = M_{11}\nu_1 + M_{12}\nu_2,
\qquad
b = M_{21}\nu_1 + M_{22}\nu_2.
$$

One skew-symmetric Coriolis-centripetal parameterization is

$$
C(\nu) =
\begin{bmatrix}
0_{3 \times 3} & -S(a) \\
-S(a) & -S(b)
\end{bmatrix}.
$$

This satisfies

$$
C(\nu) = -C^T(\nu).
$$

For rigid-body dynamics, use $M = M_{RB}$.

### Energy Property

The skew-symmetry of $C_{RB}(\nu)$ gives

$$
\nu^T C_{RB}(\nu)\nu = 0.
$$

This matters because Coriolis and centripetal forces exchange kinetic energy between degrees of freedom but do not create or remove total kinetic energy.

The rigid-body kinetic energy is

$$
T = \frac{1}{2}\nu^T M_{RB}\nu.
$$

If $\tau_{RB}=0$, then

$$
\dot{T}
=
\nu^T M_{RB}\dot{\nu}
=
-\nu^T C_{RB}(\nu)\nu
=
0.
$$

This property is heavily used in nonlinear control and observer design.

### Linear-Velocity-Independent Parameterization

For marine craft in currents, it is useful to choose a $C_{RB}$ expression that depends only on angular velocity $\nu_2$, not on the linear velocity $\nu_1$:

$$
C_{RB}^{\nu_2}(\nu) =
\begin{bmatrix}
mS(\nu_2) & -mS(\nu_2)S(r_g^b) \\
mS(r_g^b)S(\nu_2) & -S(I_b^b\nu_2)
\end{bmatrix}.
$$

This is the preferred representation when using relative velocity

$$
\nu_r = \nu - \nu_c
$$

for irrotational ocean currents. Since currents contribute linear velocity but not angular velocity, using a $C_{RB}$ that does not depend on $\nu_1$ avoids injecting current velocity into rigid-body Coriolis terms incorrectly.

## Surface-Ship 3-DOF Reduction

For a surface vessel constrained to horizontal-plane motion:

$$
\eta =
\begin{bmatrix}
x & y & \psi
\end{bmatrix}^T,
\qquad
\nu =
\begin{bmatrix}
u & v & r
\end{bmatrix}^T,
\qquad
\tau =
\begin{bmatrix}
X & Y & N
\end{bmatrix}^T.
$$

Assume the CO lies on the centerline and

$$
r_g^b =
\begin{bmatrix}
x_g & 0 & 0
\end{bmatrix}^T.
$$

The rigid-body mass matrix reduces to

$$
M_{RB}^{3DOF} =
\begin{bmatrix}
m & 0 & 0 \\
0 & m & mx_g \\
0 & mx_g & I_z
\end{bmatrix}.
$$

The linear-velocity-independent rigid-body Coriolis matrix becomes

$$
C_{RB}^{3DOF}(\nu) =
\begin{bmatrix}
0 & -mr & -mx_g r \\
mr & 0 & 0 \\
mx_g r & 0 & 0
\end{bmatrix}.
$$

Therefore

$$
M_{RB}^{3DOF}\dot{\nu} + C_{RB}^{3DOF}(\nu)\nu = \tau_{RB}
$$

expands to

$$
\begin{aligned}
m(\dot{u} - vr - x_g r^2) &= X, \\
m(\dot{v} + ur + x_g\dot{r}) &= Y, \\
I_z\dot{r} + mx_g(\dot{v} + ur) &= N.
\end{aligned}
$$

These are the rigid-body parts only. In practical maneuvering models, added mass, damping, restoring, wind, waves, current, and actuator forces are added around this core.

## How to Read the Terms

### Mass Matrix Terms

- $mI_3$: resistance to translational acceleration.
- $I_b^b$: resistance to angular acceleration about the CO.
- $\pm mS(r_g^b)$: coupling between linear and angular acceleration when CO is not at CG.

### Coriolis and Centripetal Terms

- $mS(\nu_2)\nu_1$: apparent linear acceleration caused by expressing motion in a rotating BODY frame.
- $-S(I_b^b\nu_2)\nu_2$: gyroscopic moment term.
- Offset terms using $S(r_g^b)$: inertial coupling caused by rotating around a point that is not the CG.

### Force and Moment Vector

The generalized rigid-body load is

$$
\tau_{RB} =
\begin{bmatrix}
X & Y & Z & K & M & N
\end{bmatrix}^T.
$$

The first three entries are forces:

- $X$: surge force.
- $Y$: sway force.
- $Z$: heave force.

The last three entries are moments:

- $K$: roll moment.
- $M$: pitch moment.
- $N$: yaw moment.

## Simulation Checklist

To simulate the rigid-body model:

1. Choose the CO and define $r_g^b$.
2. Define $m$ and the inertia tensor $I_g^b$ or $I_b^b$.
3. If starting from $I_g^b$, compute $I_b^b = I_g^b - mS(r_g^b)S(r_g^b)$.
4. Build $M_{RB}$.
5. Build a skew-symmetric $C_{RB}(\nu)$.
6. Compute

$$
\dot{\nu} = M_{RB}^{-1}\left[\tau_{RB} - C_{RB}(\nu)\nu\right].
$$

7. Propagate position and attitude using

$$
\dot{\eta} = J(\eta)\nu.
$$

For numerical code, avoid explicitly computing $M_{RB}^{-1}$ when possible. Solve the linear system

$$
M_{RB}\dot{\nu} = \tau_{RB} - C_{RB}(\nu)\nu
$$

instead.

## Key Takeaways

- Chapter 2 maps BODY velocities into pose rates; Chapter 3 maps forces and moments into BODY accelerations.
- The Coriolis and centripetal matrix appears because the equations are expressed in the rotating BODY frame.
- The CO is the practical modeling origin for GNC, even though the CG is the natural physical derivation point.
- The rigid-body inertia matrix is symmetric positive definite.
- $C_{RB}(\nu)$ should be parameterized as skew-symmetric so that $\nu^TC_{RB}(\nu)\nu = 0$.
- For current-relative marine craft models, prefer the $C_{RB}^{\nu_2}$ parameterization because it is independent of linear velocity.
