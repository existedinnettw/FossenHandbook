# Chapter 3 - Rigid-Body Kinetics

Kinetics is the force side of dynamics. Chapter 2 gave the geometry of motion,

$$
\dot{\eta} = J(\eta)\nu
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
M_{RB}\dot{\nu} + C_{RB}(\nu)\nu = \tau_{RB}
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

[Transport theorem](https://en.wikipedia.org/wiki/Transport_theorem) 

For any vector $a$, inertial differentiation and BODY differentiation are related by

$$
\left(\frac{d a}{dt}\right)_n
=
\left(\frac{d a}{dt}\right)_b
+ \omega_{nb}^b \times a
$$

Using the skew-symmetric operator $S(\cdot)$:

$$
\left(\frac{d a}{dt}\right)_n
=
\dot{a}^b + S(\omega_{nb}^b)a^b
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
\end{bmatrix}
$$

In this chapter:

$$
\nu_1 = v_{nb}^b,
\qquad
\nu_2 = \omega_{nb}^b
$$



## Newton-Euler Equations About the CG

[Newton-Euler equations](https://en.wikipedia.org/wiki/Newton%E2%80%93Euler_equations)

### Assumptions

The CG derivation starts from two standard marine-craft assumptions:

1. The vessel is rigid.
2. **The local NED frame is approximately inertial**, $\{n\} \approx \{i\}$.

The first assumption lets the craft be treated as one rigid body instead of as
many mass elements interacting internally. Internal forces cancel when summing
over the whole body, so only the resultant external force and moment appear.

The second assumption means that, for ordinary marine craft modeling, the
star-fixed inertial frame $\{i\}$ can be replaced by the local NED frame
$\{n\}$. Therefore,

$$
v_{ig} \approx v_{ng},
\qquad
\omega_{ig} = \omega_{ib} \approx \omega_{nb}
$$

The approximation neglects small inertial effects from Earth's rotation,

$$
\omega_{ie} = 7.2921 \times 10^{-5}\ \mathrm{rad/s}
$$

which is usually small compared with marine hydrodynamic effects.

### Vector Differentiation Rule

The proof depends on distinguishing the vector itself from the coordinates used
to represent it.

- A coordinate-free vector, such as $\vec v_{ng}$, has a physical magnitude and direction.
- A coordinate vector, such as $v_{ng}^b$, is the component representation in a chosen frame.

For any vector $\vec a$, inertial differentiation and BODY differentiation are
related by the transport theorem

$$
\left(\frac{d\vec a}{dt}\right)_n
=
\left(\frac{d\vec a}{dt}\right)_b
+ \vec\omega_{nb} \times \vec a
$$

In BODY coordinates this becomes

$$
\left[
\left(\frac{d\vec a}{dt}\right)_n
\right]^b
=
\dot a^b + S(\omega_{nb}^b)a^b
$$

where the dot denotes component differentiation in $\{b\}$ and
$S(x)y = x \times y$.

This is why extra velocity-product terms appear: the derivative is being taken
in an inertial frame, but the coordinates rotate with the body.

### Translational Motion

Newton's second law is an inertial-frame momentum balance about the CG:

$$
\vec f_g
=
\left(\frac{d\vec p_g}{dt}\right)_n,
\qquad
\vec p_g = m\vec v_{ng}
$$

Because the mass is constant,

$$
\vec f_g
=
m\left(\frac{d\vec v_{ng}}{dt}\right)_n
$$

The CG velocity can be related to the BODY origin velocity by the position
identity

$$
\vec r_{ng} = \vec r_{nb} + \vec r_{bg}
$$

Here $\vec r_{bg}$ is the vector from the BODY origin to the CG; its BODY
components are the $r_g^b$ used elsewhere in these notes.

Taking an inertial derivative,

$$
\vec v_{ng}
=
\vec v_{nb}
+ \left(\frac{d\vec r_{bg}}{dt}\right)_n
$$

For a rigid body, $\vec r_{bg}$ is fixed in BODY coordinates:

$$
\left(\frac{d\vec r_{bg}}{dt}\right)_b = 0
$$

but it is not generally fixed in the inertial frame because the BODY frame
rotates. Applying the transport theorem,

$$
\left(\frac{d\vec r_{bg}}{dt}\right)_n
=
\vec\omega_{nb} \times \vec r_{bg}
$$

Therefore,

$$
\vec v_{ng}
=
\vec v_{nb}
+ \vec\omega_{nb} \times \vec r_{bg}
$$

and, in BODY coordinates,

$$
v_{ng}^b
=
v_{nb}^b + S(\omega_{nb}^b)r_g^b
$$

For the CG equations in this section, the BODY origin is temporarily placed at
the CG. Hence $r_g^b = 0$ and

$$
v_{ng}^b = v_{nb}^b = \nu_1
$$

Now express Newton's law in BODY coordinates. The important order is:

1. take the inertial derivative,
2. then express the result in $\{b\}$.

Thus,

$$
f_g^b
=
m\left[
\left(\frac{d\vec v_{ng}}{dt}\right)_n
\right]^b
$$

Using the transport theorem again,

$$
\left[
\left(\frac{d\vec v_{ng}}{dt}\right)_n
\right]^b
=
\dot v_{ng}^b + S(\omega_{nb}^b)v_{ng}^b
$$

so

$$
m\left(\dot v_{ng}^b + S(\omega_{nb}^b)v_{ng}^b\right)
=
f_g^b
$$

At the CG, substitute $v_{ng}^b = \nu_1$ and
$\omega_{nb}^b = \nu_2$:

$$
\boxed{
m\left(\dot\nu_1 + S(\nu_2)\nu_1\right) = f_g^b
}
$$

The term $m\dot\nu_1$ is the familiar mass times acceleration term. The term
$mS(\nu_2)\nu_1 = m(\omega_{nb}^b \times v_{ng}^b)$ appears only because the
linear velocity components are written in the rotating BODY frame.

### Rotational Motion

Euler's second axiom is the angular momentum balance about the CG:

$$
\vec m_g
=
\left(\frac{d\vec h_g}{dt}\right)_n
$$

For a rigid body, angular momentum about the CG is

$$
\vec h_g = I_g\vec\omega_{nb}
$$

In BODY coordinates,

$$
h_g^b = I_g^b\omega_{nb}^b
$$

The inertia dyadic $I_g^b$ is constant in BODY coordinates because the body is
rigid and the axes are body-fixed. Therefore the BODY derivative is

$$
\left(\frac{d h_g^b}{dt}\right)_b
=
I_g^b\dot\omega_{nb}^b
$$

Apply the transport theorem to angular momentum:

$$
\left[
\left(\frac{d\vec h_g}{dt}\right)_n
\right]^b
=
\dot h_g^b + S(\omega_{nb}^b)h_g^b
$$

Substitute this into the moment balance:

$$
I_g^b\dot\omega_{nb}^b
+ S(\omega_{nb}^b)I_g^b\omega_{nb}^b
=
m_g^b
$$

Equivalently, using $a \times b = -b \times a$,

$$
I_g^b\dot\omega_{nb}^b
- S(I_g^b\omega_{nb}^b)\omega_{nb}^b
= m_g^b
$$

This is the same sign convention used on the slide:
$\omega \times (I_g\omega) = -(I_g\omega)\times\omega$.

With $\nu_2 = \omega_{nb}^b = [p,q,r]^T$,

$$
\boxed{
I_g^b\dot\nu_2 + S(\nu_2)I_g^b\nu_2 = m_g^b
}
$$

Term by term:

- $I_g^b\dot\nu_2$: moment needed for angular acceleration about the CG.
- $I_g^b\nu_2$: angular momentum about the CG, expressed in BODY coordinates.
- $S(\nu_2)I_g^b\nu_2 = \nu_2 \times (I_g^b\nu_2)$: gyroscopic coupling caused by expressing angular momentum in the rotating BODY frame.
- $m_g^b$: external moment about the CG, expressed in BODY coordinates.

If

$$
h_g^b = I_g^b\nu_2 =
\begin{bmatrix}
h_x \\ h_y \\ h_z
\end{bmatrix}
$$

then the gyroscopic term expands to

$$
S(\nu_2)I_g^b\nu_2
=
\nu_2 \times h_g^b
=
\begin{bmatrix}
q h_z - r h_y \\
r h_x - p h_z \\
p h_y - q h_x
\end{bmatrix}
$$

The moment vector is

$$
m_g^b =
\begin{bmatrix}
K_g \\ M_g \\ N_g
\end{bmatrix}
$$

so the rotational equation says

$$
\underbrace{I_g^b\dot{\nu}_2}_{\text{angular acceleration}}
+
\underbrace{S(\nu_2)I_g^b\nu_2}_{\text{gyroscopic coupling}}
=
\underbrace{m_g^b}_{\text{external moment about CG}}
$$

The inertia tensor about the CG is

$$
I_g^b =
\begin{bmatrix}
I_x & -I_{xy} & -I_{xz} \\
-I_{yx} & I_y & -I_{yz} \\
-I_{zx} & -I_{zy} & I_z
\end{bmatrix}
$$

The diagonal terms are moments of inertia:

$$
I_x = \int_V (y^2 + z^2)\rho_m\,dV,
\quad
I_y = \int_V (x^2 + z^2)\rho_m\,dV,
\quad
I_z = \int_V (x^2 + y^2)\rho_m\,dV
$$

The off-diagonal terms are products of inertia:

$$
I_{xy} = \int_V xy\rho_m\,dV,
\quad
I_{xz} = \int_V xz\rho_m\,dV,
\quad
I_{yz} = \int_V yz\rho_m\,dV
$$

with $I_{xy}=I_{yx}$, $I_{xz}=I_{zx}$, and $I_{yz}=I_{zy}$. The minus signs in
$I_g^b$ are the standard inertia-dyadic convention used in Fossen's slides.



### Matrix Form at the CG

With the coordinate origin at the CG, the generalized velocity is

$$
\nu =
\begin{bmatrix}
\nu_1 \\
\nu_2
\end{bmatrix}
=
\begin{bmatrix}
v_{ng}^b \\
\omega_{nb}^b
\end{bmatrix}
$$

so the Newton-Euler equations can be stacked as

$$
M_{RB}^{CG}\dot{\nu} + C_{RB}^{CG}(\nu)\nu = \tau_{RB}^{CG},
\qquad
\tau_{RB}^{CG} =
\begin{bmatrix}
f_g^b \\
m_g^b
\end{bmatrix}
$$

The rigid-body inertia matrix is

$$
M_{RB}^{CG} =
\begin{bmatrix}
mI_3 & 0_{3 \times 3} \\
0_{3 \times 3} & I_g^b
\end{bmatrix}
$$

Term by term:

- $mI_3$: translational inertia at the CG. It multiplies $\dot{\nu}_1$ and gives the force needed for linear acceleration.
- $I_g^b$: rotational inertia dyadic about the CG. It multiplies $\dot{\nu}_2$ and gives the moment needed for angular acceleration.
- $0_{3 \times 3}$ off-diagonal blocks: no mass-moment coupling at the CG. On the later CO slide these become $\pm mS(r_g^b)$ because the reference point is offset from the CG.

Therefore,

$$
M_{RB}^{CG}\dot{\nu}
=
\begin{bmatrix}
m\dot{\nu}_1 \\
I_g^b\dot{\nu}_2
\end{bmatrix}
$$

is just the stacked linear and angular acceleration inertia.

The Coriolis and centripetal matrix in the slide is

$$
C_{RB}^{CG}(\nu) =
\begin{bmatrix}
mS(\nu_2) & 0_{3 \times 3} \\
0_{3 \times 3} & -S(I_g^b\nu_2)
\end{bmatrix}
$$

Term by term:

- $mS(\nu_2)$: translational Coriolis/centripetal block caused by expressing the CG linear momentum in the rotating BODY frame.
- $-S(I_g^b\nu_2)$: rotational gyroscopic block. The vector $I_g^b\nu_2$ is angular momentum about the CG.
- $0_{3 \times 3}$ off-diagonal blocks: at the CG, translational velocity does not directly create a moment term, and angular velocity does not directly create a force through a lever arm. Those couplings appear after transforming to the CO.

Multiplying out the Coriolis/centripetal part gives

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
\end{bmatrix}
$$

because $S(a)b = a \times b$ and

$$
-S(I_g\omega)\omega
=
-(I_g\omega)\times \omega
=
\omega \times I_g\omega
$$

This recovers the two BODY-frame Newton-Euler equations:

$$
m(\dot{\nu}_1 + \nu_2 \times \nu_1) = f_g^b
$$

and

$$
I_g^b\dot{\nu}_2 + \nu_2 \times (I_g^b\nu_2) = m_g^b
$$

The matrix form is useful because it packages the acceleration terms in
$M_{RB}^{CG}\dot{\nu}$ and the velocity-product terms in $C_{RB}^{CG}(\nu)\nu$.



## Transforming from CG to CO

### Why Use the CO?

The CG is excellent for deriving equations, but the control system often cares about another point. Fossen's marine craft model therefore represents rigid-body equations about the BODY-fixed coordinate origin CO.

The same physical rigid body can be represented at the CO by transforming:

- velocity,
- force/moment,
- mass matrix,
- Coriolis/centripetal matrix.

### System Transformation Matrix

The $H$ matrix comes directly from the rigid-body velocity relation between
the CO and CG. Let $r_g^b$ be the vector from the CO to the CG, expressed in
BODY coordinates. The CG velocity is the CO velocity plus the velocity induced
by rotation about the CO:

$$
v_{ng}^b
=
v_{nb}^b + \omega_{nb}^b \times r_g^b
$$

Using $a \times b = -b \times a$,

$$
\omega_{nb}^b \times r_g^b
=
-r_g^b \times \omega_{nb}^b
=
-S(r_g^b)\omega_{nb}^b
=
S^T(r_g^b)\omega_{nb}^b
$$

because $S^T(r_g^b) = -S(r_g^b)$. Therefore,

$$
v_{ng}^b
=
v_{nb}^b + S^T(r_g^b)\omega_{nb}^b
$$

The angular velocity is the same physical rotation no matter whether the
reference point is the CO or CG:

$$
\omega_g^b = \omega_b^b = \omega_{nb}^b
$$

Stacking the linear and angular parts gives the slide's transformation:

$$
\begin{bmatrix}
v_{ng}^b \\
\omega_{nb}^b
\end{bmatrix}
=
H(r_g^b)
\begin{bmatrix}
v_{nb}^b \\
\omega_{nb}^b
\end{bmatrix}
$$

where the system transformation matrix is

$$
H(r_g^b) =
\begin{bmatrix}
I_3 & S^T(r_g^b) \\
0_{3 \times 3} & I_3
\end{bmatrix}
=
\begin{bmatrix}
I_3 & -S(r_g^b) \\
0_{3 \times 3} & I_3
\end{bmatrix}
$$

With the shorthand

$$
\nu_g = H(r_g^b)\nu_b
$$

this means

$$
v_g^b = v_b^b + \omega_{nb}^b \times r_g^b,
\qquad
\omega_g^b = \omega_b^b
$$

The transpose that appears on the slide is

$$
H^T(r_g^b) =
\begin{bmatrix}
I_3 & 0_{3 \times 3} \\
S(r_g^b) & I_3
\end{bmatrix}
$$

It is used to transform generalized forces and the matrix equation itself:

$$
\tau_b = H^T(r_g^b)\tau_g,
\qquad
M_{RB} = H^T(r_g^b)M_{RB}^{CG}H(r_g^b),
\qquad
C_{RB} = H^T(r_g^b)C_{RB}^{CG}H(r_g^b)
$$

The force transformation is consistent with virtual power:

$$
\tau_g^T\nu_g
=
\tau_g^TH\nu_b
=
(H^T\tau_g)^T\nu_b
=
\tau_b^T\nu_b
$$



### Parallel-Axis Theorem

The inertia tensor about the CO is

$$
I_b^b = I_g^b - mS(r_g^b)S(r_g^b)
$$

This is the compact matrix form of the [parallel-axis theorem](https://en.wikipedia.org/wiki/Parallel_axis_theorem#Moment_of_inertia_matrix).

For $r_g^b = [x_g, y_g, z_g]^T$:

$$
-S(r_g^b)S(r_g^b)
=
\|r_g^b\|^2I_3 - r_g^b(r_g^b)^T
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
\end{bmatrix}
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
\end{bmatrix}
$$

### Translational Motion About the CO

The velocity of the CG is

$$
v_g^b = \nu_1 + S(\nu_2)r_g^b
$$

The translational equation about the CO becomes

$$
m\left[
\dot{\nu}_1
- S(r_g^b)\dot{\nu}_2
+ S(\nu_2)\nu_1
- S(\nu_2)S(r_g^b)\nu_2
\right]
= f^b
$$

Equivalent vector form:

$$
m\left[
\dot{v}
+ \dot{\omega}\times r_g
+ \omega\times v
+ \omega\times(\omega\times r_g)
\right]
= f
$$

The new terms come from the offset between CO and CG:

- $\dot{\omega}\times r_g$: tangential acceleration of the CG due to angular acceleration.
- $\omega\times(\omega\times r_g)$: centripetal acceleration of the CG due to rotation.



### Rotational Motion About the CO

The rotational equation about the CO can be written compactly as the lower three rows of

$$
M_{RB}\dot{\nu} + C_{RB}(\nu)\nu = \tau_{RB}
$$

In vector form, the CO moment balance includes:

- angular acceleration through $I_b^b\dot{\omega}$,
- gyroscopic coupling through $\omega \times I_b^b\omega$,
- force/moment coupling from the offset $r_g^b$.

The important modeling lesson is that the moment equation about CO is not simply the CG moment equation with $I_g$ renamed. The offset introduces translation-rotation coupling in both $M_{RB}$ and $C_{RB}$.



### Jacobi Identity in the CO Coriolis Matrix

Slide 13 highlights a simplification in the lower-right block of the CO Coriolis and centripetal matrix. Before applying the parallel-axis theorem, the matrix obtained from $H^TC_{RB}^{CG}H$ contains

$$
C_{RB} =
\begin{bmatrix}
mS(\omega) & -mS(\omega)S(r_g) \\
mS(r_g)S(\omega) &
-mS(r_g)S(\omega)S(r_g) - S(I_g\omega)
\end{bmatrix}
$$

where $r_g = r_g^b$ and $\omega = \omega_{nb}^b$. The highlighted term is

$$
C_{RB}^{22}
=
-mS(r_g)S(\omega)S(r_g) - S(I_g\omega)
$$

At first glance this does not look like the clean gyroscopic term $-S(I_b\omega)$, because it contains the extra offset product $S(r_g)S(\omega)S(r_g)$. The slide uses the Jacobi identity to show that this extra product is exactly the correction needed to replace $I_g$ by the CO inertia $I_b$ after multiplication by $\omega$.

The vector Jacobi identity is

$$
a \times (b \times c)
+ b \times (c \times a)
+ c \times (a \times b)
= 0
$$

Choose $c = a \times b$. Then

$$
a \times (b \times (a \times b))
=
-(a \times (a \times b)) \times b
$$

In skew-matrix notation this is

$$
S(a)S(b)S(a)b
=
-S(S^2(a)b)b
$$

Now set $a = r_g$ and $b = \omega$:

$$
-S(r_g)S(\omega)S(r_g)\omega
=
S(S^2(r_g)\omega)\omega
$$

Therefore, the highlighted block multiplied by $\omega$ becomes

$$
\begin{aligned}
C_{RB}^{22}\omega
&=
\left[-mS(r_g)S(\omega)S(r_g) - S(I_g\omega)\right]\omega \\
&=
mS(S^2(r_g)\omega)\omega - S(I_g\omega)\omega \\
&=
-S\left((I_g - mS^2(r_g))\omega\right)\omega \\
&=
-S(I_b\omega)\omega
\end{aligned}
$$

because the parallel-axis theorem gives

$$
I_b = I_g - mS^2(r_g)
$$

So slide 13 is not adding a new physical force. It is proving that the ugly offset-dependent lower-right block of $C_{RB}$ is equivalent, in the moment equation, to the familiar gyroscopic term computed with the inertia tensor about the CO.



## Rigid-Body Matrix-Vector Model

### Main Equation

The rigid-body kinetics of a marine craft are written as

$$
M_{RB}\dot{\nu} + C_{RB}(\nu)\nu = \tau_{RB}
$$

Combined with Chapter 2 kinematics:

> map BODY velocities into NED position and attitude rates

$$
\dot{\eta} = J(\eta)\nu
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
M\dot{\nu} + C(\nu)\nu + D(\nu)\nu + g(\eta) = \tau + \tau_{env}
$$

where Chapter 3 contributes the rigid-body parts $M_{RB}$ and $C_{RB}$.



### Coriolis and Centripetal Matrix from a System Inertia Matrix

> **Source and scope.** This is Theorem 3.2 on slide 19 of the Chapter 3 lecture notes. 
>
> The slide does not prove the theorem; it refers the proof to
> Sagatun and Fossen (1991), *Lagrangian Formulation of Underwater Vehicles' Dynamics*. 
>
> The result below is therefore a cited general theorem, not a consequence proved by the preceding CG-to-CO calculations. A short momentum-balance derivation is included here to make the formula intelligible.

Let a constant, symmetric positive-definite 6-by-6 system inertia matrix be partitioned as

$$
M=M^T =
\begin{bmatrix}
M_{11} & M_{12} \\
M_{21} & M_{22}
\end{bmatrix},
\qquad
M_{21} = M_{12}^T,
\qquad M>0
$$

Partition the generalized velocity into translational and angular parts,

$$
\nu =
\begin{bmatrix}
\nu_1 \\
\nu_2
\end{bmatrix},
\qquad
\nu_1=[u,v,w]^T,
\qquad
\nu_2=[p,q,r]^T
$$

Then the generalized momentum is

$$
M\nu=
\begin{bmatrix}
a\\
b
\end{bmatrix},
\qquad
a = M_{11}\nu_1 + M_{12}\nu_2,
\qquad
b = M_{21}\nu_1 + M_{22}\nu_2
$$

Thus $a$ and $b$ are not arbitrary substitutions: they are respectively the linear-momentum and angular-momentum blocks expressed in BODY coordinates.

#### Short derivation

For momenta expressed in the rotating BODY frame, the Newton-Euler momentum balance has the form

$$
\begin{aligned}
\dot a + \nu_2\times a &= f,\\
\dot b + \nu_1\times a + \nu_2\times b &= \tau.
\end{aligned}
$$

Because $M$ is constant, $[\dot a^T,\dot b^T]^T=M\dot\nu$. The remaining velocity-product terms must therefore equal $C(\nu)\nu$. Using $S(x)y=x\times y$ and $-S(x)y=y\times x$ gives

$$
\begin{aligned}
\begin{bmatrix}
\nu_2\times a\\
\nu_1\times a+\nu_2\times b
\end{bmatrix}
&=
\begin{bmatrix}
-S(a)\nu_2\\
-S(a)\nu_1-S(b)\nu_2
\end{bmatrix}\\
&=
\underbrace{
\begin{bmatrix}
0_{3 \times 3} & -S(a) \\
-S(a) & -S(b)
\end{bmatrix}}_{C(\nu)}
\begin{bmatrix}
\nu_1\\
\nu_2
\end{bmatrix}.
\end{aligned}
$$

Hence one skew-symmetric Coriolis-centripetal parameterization is

$$
C(\nu) =
\begin{bmatrix}
0_{3 \times 3} & -S(a) \\
-S(a) & -S(b)
\end{bmatrix}
$$

This satisfies

$$
C(\nu) = -C^T(\nu)
$$

The parameterization is not unique: the equations determine the product $C(\nu)\nu$, so another matrix giving the same product is dynamically
equivalent. The form above is useful because skew-symmetry is explicit.

For rigid-body dynamics, set $M=M_{RB}$. The next Lagrangian parameterization is obtained by substituting the rigid-body blocks into this theorem.

**Reference:** S. I. Sagatun and T. I. Fossen (1991), "Lagrangian Formulation of Underwater Vehicles' Dynamics," *IEEE International Conference on Systems,
Man, and Cybernetics*, pp. 1029-1034.



### Lagrangian Parameterization: Applying Theorem 3.2

This is not a separate result. It is Theorem 3.2 evaluated with the rigid-body inertia matrix

$$
M_{RB}=
\begin{bmatrix}
mI_3 & -mS(r_g^b)\\
mS(r_g^b) & I_b^b
\end{bmatrix}.
$$

Writing $v=\nu_1$, $\omega=\nu_2$, and $r_g=r_g^b$ for compactness, the two momentum blocks in the theorem become

$$
\begin{aligned}
a
&=mv-mS(r_g)\omega
=mv+mS(\omega)r_g,\\
b
&=mS(r_g)v+I_b^b\omega
=-mS(v)r_g+I_b^b\omega.
\end{aligned}
$$

Substitute these expressions into $C(\nu)=[\begin{smallmatrix}0&-S(a)\\-S(a)&-S(b)\end{smallmatrix}]$ and use the Jacobi identity together with $S(x)x=0$. 

After collecting dynamically equivalent terms, the result is the **Lagrangian parameterization** shown on slide 21:
$$
C_{RB}^{L}(\nu)=
\begin{bmatrix}
0_{3\times3}
&-mS(v)-mS(\omega)S(r_g)\\
-mS(v)+mS(r_g)S(\omega)
&-S(I_b^b\omega)
\end{bmatrix}.
$$

It is skew-symmetric because

$$
\left[-mS(v)-mS(\omega)S(r_g)\right]^T
=mS(v)-mS(r_g)S(\omega),
$$

which is the negative of the lower-left block. Thus the theorem supplies the general construction, while substituting the four blocks of $M_{RB}$ supplies this rigid-body-specific matrix.



### Energy Property

The skew-symmetry of $C_{RB}(\nu)$ gives

$$
\nu^T C_{RB}(\nu)\nu = 0
$$

This matters because Coriolis and centripetal forces exchange kinetic energy between degrees of freedom but do not create or remove total kinetic energy.

The rigid-body kinetic energy is

$$
T = \frac{1}{2}\nu^T M_{RB}\nu
$$

If $\tau_{RB}=0$, then

$$
\dot{T}
=
\nu^T M_{RB}\dot{\nu}
=
-\nu^T C_{RB}(\nu)\nu
=
0
$$

This property is heavily used in nonlinear control and observer design.



### Linear-Velocity-Independent Parameterization

The Lagrangian matrix above still contains $v=\nu_1$. Slide 23 obtains an equivalent matrix with no $v$ in its entries. The key is that the dynamics
depend on the product $C_{RB}(\nu)\nu$, not on the individual blocks of $C_{RB}$.

Consider the upper block row of the Lagrangian product:

$$
\begin{aligned}
\left[-mS(v)-mS(\omega)S(r_g)\right]\omega
&=mS(\omega)v-mS(\omega)S(r_g)\omega\\
&=
\begin{bmatrix}
mS(\omega)&-mS(\omega)S(r_g)
\end{bmatrix}
\begin{bmatrix}
v\\
\omega
\end{bmatrix},
\end{aligned}
$$

where the decisive cross-product identity is

$$
-S(v)\omega=S(\omega)v.
$$

Therefore the $-mS(v)$ term can be moved from block $C_{12}$ to block $C_{11}$ as $mS(\omega)$. In the lower block row, the remaining
$-mS(v)$ term contributes nothing because
$$
-mS(v)v=0.
$$

It can therefore be deleted from $C_{21}$. The other blocks are unchanged.
Consequently,
$$
C_{RB}^{L}(\nu)\nu=C_{RB}^{\nu_2}(\nu)\nu,
$$

even though the two matrices are not equal entry by entry. The resulting linear-velocity-independent parameterization is

$$
C_{RB}^{\nu_2}(\nu) =
\begin{bmatrix}
mS(\nu_2) & -mS(\nu_2)S(r_g^b) \\
mS(r_g^b)S(\nu_2) & -S(I_b^b\nu_2)
\end{bmatrix}
$$

This removes linear velocity from the matrix without changing the Coriolis-centripetal force vector. The new matrix also remains skew-symmetric.

This is the preferred representation when using relative velocity

$$
\nu_r = \nu - \nu_c
$$

for irrotational ocean currents. Since currents contribute linear velocity but not angular velocity, using a $C_{RB}$ that does not depend on $\nu_1$ avoids injecting current velocity into rigid-body Coriolis terms incorrectly.





### What "Linearized 6-DOF" Means

**Linearized** means replacing the nonlinear equations by their first-order Taylor approximation near a chosen nominal motion, also called an operating
point. It does **not** mean reducing the number of degrees of freedom: all six velocity perturbations remain in the model.

#### Why Linearize?

The rigid-body kinetics in this chapter are nonlinear mainly because $C_{RB}(\nu)\nu$ contains products of velocities, such as $ur$, $uq$, $pq$, and $r^2$. 

A linear model near a specified motion is useful because it permits:

- eigenvalue and local stability analysis;
- transfer-function and frequency-response analysis;
- classical control design and tuning;
- systematic linear methods such as pole placement, LQR, and Kalman filters;
- easier identification of coupled modes such as sway-yaw and roll-yaw;
- simpler and faster simulation or real-time implementation.

The approximation retains the effect of the nominal motion while removing
products of small deviations. For example, near constant surge speed $U$,

$$
u=U+\delta u,
\qquad
r=\delta r,
$$

so

$$
ur=(U+\delta u)\delta r
=U\delta r+\delta u\,\delta r
\approx U\delta r.
$$

The term $U\delta r$ is first order and is retained because $U$ is a fixed nominal value. The product $\delta u\,\delta r$ is second order in the small perturbations and is discarded. The resulting equations are linear in the perturbation variables even though they still capture how forward speed affects the local dynamics.

Linearization is therefore a trade-off: it gives a much simpler model for analysis and controller design, but only near the selected operating point.
The nonlinear model should be used for large attitude changes, aggressive maneuvers, large velocity deviations, or operation over a wide speed range.

On slide 26, the nominal motion is steady surge at constant forward speed
$U$:

$$
\nu_0=
\begin{bmatrix}
U&0&0&0&0&0
\end{bmatrix}^T.
$$

Write the actual velocity and load as a nominal value plus a small perturbation,

$$
\nu=\nu_0+\delta\nu,
\qquad
\tau_{RB}=\tau_{RB,0}+\delta\tau_{RB}.
$$

Define the nonlinear Coriolis-centripetal force vector

$$
h(\nu)=C_{RB}(\nu)\nu.
$$

Its first-order Taylor expansion is

$$
h(\nu_0+\delta\nu)
\approx
h(\nu_0)
+
\left.\frac{\partial h}{\partial\nu}\right|_{\nu_0}\delta\nu.
$$

After subtracting the nominal equation, the perturbation dynamics are

$$
M_{RB}\,\delta\dot\nu
+C_{RB}^*\delta\nu
=\delta\tau_{RB},
\qquad
C_{RB}^*
:=
\left.\frac{\partial[C_{RB}(\nu)\nu]}{\partial\nu}\right|_{\nu_0}
$$

The slide omits the $\delta$ symbols and reuses $\nu$ and $\tau_{RB}$ for the perturbation variables. This is common shorthand, but the displayed
linearized equation is a model of deviations from $\nu_0$, not the original absolute variables.

For this particular operating point, slide 26 writes the constant linearized matrix as

$$
C_{RB}^*=UM_{RB}L,
$$

where the only nonzero entries of $L$ are

$$
L_{2,6}=1,
\qquad
L_{3,5}=-1.
$$

Hence

$$
L\,\delta\nu=
\begin{bmatrix}
0&\delta r&-\delta q&0&0&0
\end{bmatrix}^T.
$$

Terms such as $U\,\delta r$ and $U\,\delta q$ remain because $U$ is the fixed nominal speed, not a small perturbation. Products of two perturbations, such
as $\delta u\,\delta r$ or $\delta q\,\delta r$, are second order and are discarded.

> **Important:** $C_{RB}^*$ is the Jacobian of the complete vector $C_{RB}(\nu)\nu$; it is not generally $C_{RB}(\nu_0)$. Also, $C_{RB}^*$ need not be skew-symmetric. Slides 26-27 explicitly note that linearization destroys that matrix property.

The approximation is accurate only for sufficiently small deviations around the chosen forward speed $U$. A substantially different speed or operating
condition requires a new linearization or use of the nonlinear model.




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
\end{bmatrix}^T
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
\dot{\nu} = M_{RB}^{-1}\left[\tau_{RB} - C_{RB}(\nu)\nu\right]
$$

7. Propagate position and attitude using

$$
\dot{\eta} = J(\eta)\nu
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
