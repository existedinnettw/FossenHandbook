# Chapter 2 - Kinematics

Kinematics is the geometry of motion: position, attitude, velocity, and coordinate transformations. Kinetics comes later and studies the forces and moments that cause motion.

This note is organized by concept instead of slide order.

## Overview

- **Frames** tell us where vectors are expressed: ECI, ECEF, NED, BODY, FLOW.
- **State vectors** collect position, attitude, and velocity into compact notation.
- **Rigid-body kinematics** maps BODY velocities to NED position and attitude rates.
- **Attitude representations** can use Euler angles or unit quaternions.
- **Navigation coordinates** convert between ECEF, longitude/latitude/height, and local flat-Earth NED.
- **Relative-flow geometry** defines course, heading, crab, angle of attack, sideslip, and FLOW axes.

## Foundations

### Notation First

Superscripts describe the frame where a vector is expressed:

- $b$: BODY frame.
- $n$: NED frame.
- $e$: ECEF frame.

The subscript $nb$ means "BODY with respect to NED":

- $\omega_{nb}^b$: angular velocity of BODY relative to NED, expressed in BODY.
  - rotational velocity of frame $\{b\}$ with respect to frame $\{n\}$.

- $v_{nb}^b$: linear velocity of BODY origin relative to NED origin, expressed in BODY.
  - translational velocity of the origin of $\{b\}$ with respect to the origin of $\{n\}$.

- $\Theta_{nb}$: Euler attitude of BODY relative to NED.
- $p_{nb}^n$: position from NED origin to BODY origin, expressed in NED.

The skew-symmetric cross-product operator is:

$$
a \times b = S(a)b
$$

where

$$
S(a) =
\begin{bmatrix}
0 & -a_3 & a_2 \\
a_3 & 0 & -a_1 \\
-a_2 & a_1 & 0
\end{bmatrix}
$$

### Reference Frames

#### ECI: Earth-Centered Inertial Frame `{i}`

$$
\{i\} = (x_i, y_i, z_i)
$$

ECI is treated as a non-accelerating inertial frame for terrestrial navigation.

#### ECEF: Earth-Centered Earth-Fixed Frame `{e}`

$$
\{e\} = (x_e, y_e, z_e)
$$

The origin is at Earth's center, and the axes rotate with Earth.

- $x_e$: equatorial plane, pointing toward the prime meridian.
- $y_e$: equatorial plane, completing the right-handed frame.
- $z_e$: Earth's rotation axis.

Earth rotation rate:

$$
\omega_{ie} = 7.2921 \times 10^{-5}\ \mathrm{rad/s}
$$

#### NED: North-East-Down Frame `{n}`

[wiki Local tangent plane coordinates](https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates)
$$
\{n\} = (x_n, y_n, z_n)
$$

- $x_n$: true North.
- $y_n$: East.
- $z_n$: Down, normal to Earth's surface.

NED is usually a tangent plane on the WGS-84 ellipsoid.

- **Terrestrial navigation**: NED moves with the craft using time-varying longitude and latitude $(l,\mu)$.
- **Local navigation**: NED is fixed at a local origin $(l_0,\mu_0)$.

Flat-Earth navigation is a local approximation, roughly suitable for small areas such as $10\ \mathrm{km} \times 10\ \mathrm{km}$.

#### BODY: Body-Fixed Frame `{b}`

$$
\{b\} = (x_b, y_b, z_b)
$$

For marine craft:

- $x_b$: longitudinal axis, aft to fore.
- $y_b$: transversal axis, to starboard.
- $z_b$: normal axis, top to bottom.

#### FLOW: Body-Fixed Flow-Axes Frame `{f}`

FLOW aligns its $x$-axis with the relative flow velocity. This makes hydrodynamic/aerodynamic force data easier to use:

- lift is perpendicular to relative flow,
- drag is parallel to relative flow,
- lookup tables are often expressed using angle of attack $\alpha$ and sideslip $\beta$.

### Marine Craft State Vectors

For a 6-DOF marine craft, the generalized position is:

$$
\eta =
\begin{bmatrix}
x^n & y^n & z^n & \phi & \theta & \psi
\end{bmatrix}^T
$$

where:

- $x^n,y^n,z^n$: position in NED.
- $\phi$: roll.
- $\theta$: pitch.
- $\psi$: yaw/heading.

The BODY-fixed velocity vector is:

$$
\nu =
\begin{bmatrix}
u & v & w & p & q & r
\end{bmatrix}^T
$$

where:

- $u,v,w$: surge, sway, heave velocities.
- $p,q,r$: roll, pitch, yaw angular velocities.

6-DOF naming:

| DOF | Motion | Velocity | Force/moment |
| --- | --- | --- | --- |
| 1 | Surge | $u$ | $X$ |
| 2 | Sway | $v$ | $Y$ |
| 3 | Heave | $w$ | $Z$ |
| 4 | Roll | $p$ | $K$ |
| 5 | Pitch | $q$ | $M$ |
| 6 | Yaw | $r$ | $N$ |

Useful body-fixed reference points:

- **CO**: coordinate origin $o_b$ of the body-fixed frame $\{b\}$.

The following time-varying points are expressed with respect to the CO  

- CG: center of gravity
- CB: center of buoyancy
  - The center of buoyancy is the exact center of the underwater volume of a boat. It is the single point where all the upward forces of water act.
- **CF**: center of flotation, the centroid of the calm-water waterplane area $A_{wp}$.
  - The center of flotation is the center of the **waterplane area**. For small-angle linear theory, it is the pivot point where the ship rolling and pitching.

## Rigid-Body Kinematics

### Core Kinematic Model

The main goal is to map BODY velocities into NED position and attitude rates:

$$
\dot{\eta} = J(\eta)\nu
$$

with

$$
J(\eta) =
\begin{bmatrix}
R_b^n(\Theta_{nb}) & 0_{3 \times 3} \\
0_{3 \times 3} & T_\Theta(\Theta_{nb})
\end{bmatrix}
$$

The translational part is:

$$
\dot{p}_{nb}^n = R_b^n v_{nb}^b
$$

The angular part is:

$$
\dot{\Theta}_{nb} = T_\Theta(\Theta_{nb})\omega_{nb}^b
$$

where:

$$
v_{nb}^b =
\begin{bmatrix}
u & v & w
\end{bmatrix}^T,
\qquad
\omega_{nb}^b =
\begin{bmatrix}
p & q & r
\end{bmatrix}^T
$$

### Rotation Basics

[Euler's rotation theorem](https://en.wikipedia.org/wiki/Euler%27s_rotation_theorem) says that any 3-D rotation can be described by one angle $\omega$ about one unit axis $\epsilon$ (so called [Axis–angle representation](https://en.wikipedia.org/wiki/Axis–angle_representation)):
$$
\epsilon =
\begin{bmatrix}
\epsilon_1 & \epsilon_2 & \epsilon_3
\end{bmatrix}^T,
\qquad
\epsilon^T\epsilon = 1
$$

The axis $\epsilon$ is the line that stays fixed during the rotation. The angle $\omega$ is positive by the right-hand rule about that axis.

The skew-symmetric matrix of the rotation axis is:

$$
S(\epsilon) =
\begin{bmatrix}
0 & -\epsilon_3 & \epsilon_2 \\
\epsilon_3 & 0 & -\epsilon_1 \\
-\epsilon_2 & \epsilon_1 & 0
\end{bmatrix}
$$

so that $S(\epsilon)x = \epsilon \times x$ for any vector $x$. Its square is:

$$
S^2(\epsilon) =
\begin{bmatrix}
\epsilon_1^2-1 & \epsilon_1\epsilon_2 & \epsilon_1\epsilon_3 \\
\epsilon_1\epsilon_2 & \epsilon_2^2-1 & \epsilon_2\epsilon_3 \\
\epsilon_1\epsilon_3 & \epsilon_2\epsilon_3 & \epsilon_3^2-1
\end{bmatrix}
= \epsilon\epsilon^T - I_3
$$

because $\epsilon$ is a unit vector. This identity is useful because it shows the geometry:

- $I_3$ keeps the original vector.
- $\sin\omega S(\epsilon)$ rotates the component perpendicular to the axis by $90^\circ$ in the positive direction.
- $(1-\cos\omega)S^2(\epsilon)$ corrects the perpendicular component so the final vector has the right angle $\omega$.

For a vector $x$, Rodrigues' formula can also be read as:

$$
x' =
x\cos\omega
+ (\epsilon \times x)\sin\omega
+ \epsilon(\epsilon^Tx)(1-\cos\omega)
$$

The last term preserves the component of $x$ parallel to the axis. If $x$ is exactly parallel to $\epsilon$, then $\epsilon \times x=0$ and the rotation does not change it.

Rotation matrix

$$
R_{\omega,\epsilon}
= I_3 + \sin\omega S(\epsilon)
+ (1-\cos\omega)S^2(\epsilon)
$$

This is [Rodrigues' rotation formula](https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula).

Using $c=\cos\omega$, $s=\sin\omega$, and $\gamma=1-\cos\omega$, the full matrix is:

$$
R_{\omega,\epsilon} =
\begin{bmatrix}
c+\epsilon_1^2\gamma &
\epsilon_1\epsilon_2\gamma-\epsilon_3s &
\epsilon_1\epsilon_3\gamma+\epsilon_2s \\
\epsilon_2\epsilon_1\gamma+\epsilon_3s &
c+\epsilon_2^2\gamma &
\epsilon_2\epsilon_3\gamma-\epsilon_1s \\
\epsilon_3\epsilon_1\gamma-\epsilon_2s &
\epsilon_3\epsilon_2\gamma+\epsilon_1s &
c+\epsilon_3^2\gamma
\end{bmatrix}
$$

This is the same object as the rotation matrices used later for BODY and NED frames. The notation changes depending on what the rotation maps:

$$
x^n = R_b^n x^b,
\qquad
x^b = R_n^b x^n = (R_b^n)^T x^n
$$

Here $R_b^n$ means "take the coordinates of a vector expressed in BODY and express the same physical vector in NED."



#### Applying Rodrigues to BODY/NED Velocity

If BODY is rotated from NED by angle $\omega$ about axis $\epsilon$, the BODY-to-NED rotation matrix can be written directly as:

$$
R_b^n = R_{\omega,\epsilon}
$$

Then the BODY velocity vector

$$
v_{nb}^b =
\begin{bmatrix}
u & v & w
\end{bmatrix}^T
$$

is transformed to NED by:

$$
v_{nb}^n =
\dot{p}_{nb}^n =
R_b^n v_{nb}^b
$$

In full component form:

$$
\begin{bmatrix}
\dot{x}^n \\
\dot{y}^n \\
\dot{z}^n
\end{bmatrix}
=
\begin{bmatrix}
c+\epsilon_1^2\gamma &
\epsilon_1\epsilon_2\gamma-\epsilon_3s &
\epsilon_1\epsilon_3\gamma+\epsilon_2s \\
\epsilon_2\epsilon_1\gamma+\epsilon_3s &
c+\epsilon_2^2\gamma &
\epsilon_2\epsilon_3\gamma-\epsilon_1s \\
\epsilon_3\epsilon_1\gamma-\epsilon_2s &
\epsilon_3\epsilon_2\gamma+\epsilon_1s &
c+\epsilon_3^2\gamma
\end{bmatrix}
\begin{bmatrix}
u \\
v \\
w
\end{bmatrix}
$$

Therefore:

$$
\dot{x}^n =
(c+\epsilon_1^2\gamma)u
+(\epsilon_1\epsilon_2\gamma-\epsilon_3s)v
+(\epsilon_1\epsilon_3\gamma+\epsilon_2s)w
$$

$$
\dot{y}^n =
(\epsilon_2\epsilon_1\gamma+\epsilon_3s)u
+(c+\epsilon_2^2\gamma)v
+(\epsilon_2\epsilon_3\gamma-\epsilon_1s)w
$$

$$
\dot{z}^n =
(\epsilon_3\epsilon_1\gamma-\epsilon_2s)u
+(\epsilon_3\epsilon_2\gamma+\epsilon_1s)v
+(c+\epsilon_3^2\gamma)w
$$

The inverse direction uses the transpose:

$$
v_{nb}^b = R_n^b v_{nb}^n = (R_b^n)^T v_{nb}^n
$$

For a pure yaw rotation $\psi$ about the NED down axis,

$$
\epsilon =
\begin{bmatrix}
0 & 0 & 1
\end{bmatrix}^T,
\qquad
\omega=\psi
$$

Rodrigues gives:

$$
R_b^n =
\begin{bmatrix}
\cos\psi & -\sin\psi & 0 \\
\sin\psi & \cos\psi & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

and:

$$
\begin{bmatrix}
\dot{x}^n \\
\dot{y}^n \\
\dot{z}^n
\end{bmatrix}
=
\begin{bmatrix}
u\cos\psi - v\sin\psi \\
u\sin\psi + v\cos\psi \\
w
\end{bmatrix}
$$

So if the craft has only surge velocity $u>0$, its NED velocity points along the heading direction:

$$
\dot{x}^n = u\cos\psi,
\qquad
\dot{y}^n = u\sin\psi
$$



### Euler-Angle Attitude Representation

Marine craft and aircraft usually use the Tait-Bryan ZYX sequence:

1. Yaw $\psi$ about $z$.
2. Pitch $\theta$ about $y$.
3. Roll $\phi$ about $x$.

The Euler angle vector (attitude) is:

$$
\Theta_{nb} =
\begin{bmatrix}
\phi & \theta & \psi
\end{bmatrix}^T
$$

The BODY-to-NED rotation matrix is:

$$
R_b^n := R(\Theta_{nb}) = R_{z,\psi}R_{y,\theta}R_{x,\phi}
$$

and:

$$
(R_b^n)^{-1} = R_n^b = (R_b^n)^T
$$

Using $c(\cdot)=\cos(\cdot)$, $s(\cdot)=\sin(\cdot)$, and $t(\cdot)=\tan(\cdot)$:

$$
R_b^n =
\begin{bmatrix}
c\psi c\theta &
-s\psi c\phi + c\psi s\theta s\phi &
s\psi s\phi + c\psi c\phi s\theta \\
s\psi c\theta &
c\psi c\phi + s\phi s\theta s\psi &
-c\psi s\phi + s\theta s\psi c\phi \\
-s\theta &
c\theta s\phi &
c\theta c\phi
\end{bmatrix}
$$

Linear velocity transformation from BODY to NED:

$$
\begin{bmatrix}
\dot{x}^n \\
\dot{y}^n \\
\dot{z}^n
\end{bmatrix}
=
R_b^n
\begin{bmatrix}
u \\
v \\
w
\end{bmatrix}
$$

or, term by term:

$$
\dot{x}^n =
c\psi c\theta u
+(-s\psi c\phi + c\psi s\theta s\phi)v
+(s\psi s\phi + c\psi c\phi s\theta)w
$$

$$
\dot{y}^n =
s\psi c\theta u
+(c\psi c\phi + s\phi s\theta s\psi)v
+(-c\psi s\phi + s\theta s\psi c\phi)w
$$

$$
\dot{z}^n =
-s\theta u
+c\theta s\phi v
+c\theta c\phi w
$$

The inverse linear velocity transformation is:

$$
v_{nb}^b =
\begin{bmatrix}
u & v & w
\end{bmatrix}^T
=
R_n^b\dot{p}_{nb}^n
=
(R_b^n)^T
\begin{bmatrix}
\dot{x}^n & \dot{y}^n & \dot{z}^n
\end{bmatrix}^T
$$

Small-angle approximation:

$$
R(\delta\Theta_{nb}) \approx I_3 + S(\delta\Theta_{nb})
=
\begin{bmatrix}
1 & -\delta\psi & \delta\theta \\
\delta\psi & 1 & -\delta\phi \\
-\delta\theta & \delta\phi & 1
\end{bmatrix}
$$

Euler angle rates are related to BODY angular velocity by:

$$
\dot{\Theta}_{nb} = T_\Theta(\Theta_{nb})\omega_{nb}^b
$$

where

$$
T_\Theta(\Theta_{nb}) =
\begin{bmatrix}
1 & s\phi t\theta & c\phi t\theta \\
0 & c\phi & -s\phi \\
0 & s\phi / c\theta & c\phi / c\theta
\end{bmatrix}
$$

and

$$
T_\Theta^{-1}(\Theta_{nb}) =
\begin{bmatrix}
1 & 0 & -s\theta \\
0 & c\phi & c\theta s\phi \\
0 & -s\phi & c\theta c\phi
\end{bmatrix}
$$

Therefore the Euler-angle rate equations are:

$$
\dot{\phi} = p + s\phi t\theta q + c\phi t\theta r
$$

$$
\dot{\theta} = c\phi q - s\phi r
$$

$$
\dot{\psi} = \frac{s\phi}{c\theta}q + \frac{c\phi}{c\theta}r
$$

Putting the linear and angular parts together gives the **6-DOF kinematic equations**:

$$
\begin{bmatrix}
\dot{x}^n \\
\dot{y}^n \\
\dot{z}^n \\
\dot{\phi} \\
\dot{\theta} \\
\dot{\psi}
\end{bmatrix}
=
\begin{bmatrix}
c\psi c\theta u
+(-s\psi c\phi + c\psi s\theta s\phi)v
+(s\psi s\phi + c\psi c\phi s\theta)w \\
s\psi c\theta u
+(c\psi c\phi + s\phi s\theta s\psi)v
+(-c\psi s\phi + s\theta s\psi c\phi)w \\
-s\theta u
+c\theta s\phi v
+c\theta c\phi w \\
p + s\phi t\theta q + c\phi t\theta r \\
c\phi q - s\phi r \\
\dfrac{s\phi}{c\theta}q + \dfrac{c\phi}{c\theta}r
\end{bmatrix}
$$

Euler angles are intuitive but singular at:

$$
\theta = \pm 90^\circ
$$

For sampled simulation or digital control, the continuous-time kinematics can be discretized with sampling time $h$. A simple forward-Euler update is:

$$
\eta[k+1] =
\eta[k] + hJ(\eta[k])\nu[k]
$$

This means the rotation matrices are evaluated at the current attitude $\Theta_{nb}[k]=[\phi[k],\theta[k],\psi[k]]^T$, and the BODY velocity is assumed approximately constant over the interval $[kh,(k+1)h)$.

The position part is:

$$
p_{nb}^n[k+1] = p_{nb}^n[k] + hR_b^n[k]v_{nb}^b[k]
$$

or:

$$
\begin{bmatrix}
x^n[k+1] \\
y^n[k+1] \\
z^n[k+1]
\end{bmatrix}
=
\begin{bmatrix}
x^n[k] \\
y^n[k] \\
z^n[k]
\end{bmatrix}
+ h
\begin{bmatrix}
\dot{x}^n[k] \\
\dot{y}^n[k] \\
\dot{z}^n[k]
\end{bmatrix}
$$

The attitude part is:

$$
\Theta_{nb}[k+1] =
\Theta_{nb}[k] + hT_\Theta(\Theta_{nb}[k])\omega_{nb}^b[k]
$$

Forward Euler is easy to read but can accumulate attitude and position error when $h$ is large or the craft rotates quickly. The slides recommend higher-order integration, such as RK4 (`rk45`, `ode45`), for better numerical behavior.



### Quaternion Attitude Representation

Unit [quaternions](https://en.wikipedia.org/wiki/Quaternion) avoid Euler-angle singularity by using four parameters:

$$
q =
\begin{bmatrix}
\eta & \epsilon^T
\end{bmatrix}^T
=
\begin{bmatrix}
\eta & \epsilon_1 & \epsilon_2 & \epsilon_3
\end{bmatrix}^T
$$

with unit constraint:

$$
\eta^2 + \epsilon_1^2 + \epsilon_2^2 + \epsilon_3^2 = 1
$$

For a rotation angle $\beta$ about unit axis $\lambda$:

$$
\eta = \cos \frac{\beta}{2},
\qquad
\epsilon = \lambda \sin \frac{\beta}{2}
$$

The [quaternion rotation matrix](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Quaternion-derived_rotation_matrix) is:

$$
R(q_b^n) = I_3 + 2\eta S(\epsilon) + 2S^2(\epsilon)
$$

Here $S(\epsilon)$ is the skew-symmetric matrix built from the vector part of the quaternion:

$$
S(\epsilon) =
\begin{bmatrix}
0 & -\epsilon_3 & \epsilon_2 \\
\epsilon_3 & 0 & -\epsilon_1 \\
-\epsilon_2 & \epsilon_1 & 0
\end{bmatrix}
$$

It is a matrix way to write a cross product:

$$
S(\epsilon)x = \epsilon \times x
$$

In the quaternion section, $\epsilon$ is not the unit rotation axis itself. The axis is $\lambda$, while:

$$
\epsilon = \lambda \sin \frac{\beta}{2}
$$

so $\epsilon$ is the axis scaled by the half-angle sine. Also, $S^2(\epsilon)$ means:

$$
S^2(\epsilon) = S(\epsilon)S(\epsilon)
$$

or:

$$
R(q_b^n) =
\begin{bmatrix}
1 - 2(\epsilon_2^2+\epsilon_3^2) &
2(\epsilon_1\epsilon_2-\epsilon_3\eta) &
2(\epsilon_1\epsilon_3+\epsilon_2\eta) \\
2(\epsilon_1\epsilon_2+\epsilon_3\eta) &
1 - 2(\epsilon_1^2+\epsilon_3^2) &
2(\epsilon_2\epsilon_3-\epsilon_1\eta) \\
2(\epsilon_1\epsilon_3-\epsilon_2\eta) &
2(\epsilon_2\epsilon_3+\epsilon_1\eta) &
1 - 2(\epsilon_1^2+\epsilon_2^2)
\end{bmatrix}
$$

Linear velocity:

$$
\dot{p}_{nb}^n = R(q_b^n)v_{nb}^b
$$



Quaternion rate:
$$
\dot{q}_b^n = T(q_b^n)\omega_{nb}^b
$$

Here $T(q_b^n)$ is kinematic transformation from BODY angular velocity to quaternion rate:

$$
\omega_{nb}^b =
\begin{bmatrix}
p & q & r
\end{bmatrix}^T
\quad \Longrightarrow \quad
\dot{q}_b^n =
\begin{bmatrix}
\dot{\eta} & \dot{\epsilon}_1 & \dot{\epsilon}_2 & \dot{\epsilon}_3
\end{bmatrix}^T
$$

The angular velocity has three components, but the quaternion has four components. Therefore $T(q_b^n)$ is a $4 \times 3$ matrix. It tells how the four quaternion parameters must change when the craft rotates with BODY angular velocity $\omega_{nb}^b$.

This comes from writing angular velocity as a pure quaternion:

$$
\omega_q =
\begin{bmatrix}
0 & \omega^b_{nb}
\end{bmatrix}^T
=
\begin{bmatrix}
0 & p & q & r
\end{bmatrix}^T
$$

and using the quaternion differential equation:

> [Rotation Quaternions, and How to Use Them](https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html)

$$
\dot{q}_b^n = \frac{1}{2} q_b^n \otimes \omega_q
$$

* \(\otimes \): [Quaternion multiplication](https://www.mathworks.com/help/aeroblks/quaternionmultiplication.html)

Expanding this quaternion product gives the matrix $T(q_b^n)$ below.

The matrix is:

$$
T(q_b^n) =
\frac{1}{2}
\begin{bmatrix}
-\epsilon_1 & -\epsilon_2 & -\epsilon_3 \\
\eta & -\epsilon_3 & \epsilon_2 \\
\epsilon_3 & \eta & -\epsilon_1 \\
-\epsilon_2 & \epsilon_1 & \eta
\end{bmatrix}
$$

Component form:

$$
\begin{aligned}
\dot{\eta} &= -\frac{1}{2}(\epsilon_1p+\epsilon_2q+\epsilon_3r) \\
\dot{\epsilon}_1 &= \frac{1}{2}(\eta p-\epsilon_3q+\epsilon_2r) \\
\dot{\epsilon}_2 &= \frac{1}{2}(\epsilon_3p+\eta q-\epsilon_1r) \\
\dot{\epsilon}_3 &= \frac{1}{2}(-\epsilon_2p+\epsilon_1q+\eta r)
\end{aligned}
$$

Quaternion-based kinematics use seven ODEs:

$$
\begin{bmatrix}
\dot{p}_{nb}^n \\
\dot{q}_b^n
\end{bmatrix}
=
\begin{bmatrix}
R(q_b^n) & 0_{3 \times 3} \\
0_{4 \times 3} & T(q_b^n)
\end{bmatrix}
\begin{bmatrix}
v_{nb}^b \\
\omega_{nb}^b
\end{bmatrix}
$$

This is nonsingular, but one extra ODE is needed and the unit constraint must be preserved.

### Euler-Quaternion Conversion

[Conversion between quaternions and Euler angles](https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)

For $\Theta_{nb}=[\phi,\theta,\psi]^T$:

$$
q_b^n =
\begin{bmatrix}
\cos\frac{\psi}{2}\cos\frac{\theta}{2}\cos\frac{\phi}{2}
+\sin\frac{\psi}{2}\sin\frac{\theta}{2}\sin\frac{\phi}{2} \\
\cos\frac{\psi}{2}\cos\frac{\theta}{2}\sin\frac{\phi}{2}
-\sin\frac{\psi}{2}\sin\frac{\theta}{2}\cos\frac{\phi}{2} \\
\sin\frac{\psi}{2}\cos\frac{\theta}{2}\sin\frac{\phi}{2}
+\cos\frac{\psi}{2}\sin\frac{\theta}{2}\cos\frac{\phi}{2} \\
\sin\frac{\psi}{2}\cos\frac{\theta}{2}\cos\frac{\phi}{2}
-\cos\frac{\psi}{2}\sin\frac{\theta}{2}\sin\frac{\phi}{2}
\end{bmatrix}
$$

Euler angles can be recovered from a rotation matrix:

$$
\phi = \operatorname{atan2}(R_{32},R_{33})
$$

$$
\theta = -\sin^{-1}(R_{31}), \qquad \theta \ne \pm 90^\circ
$$

$$
\psi = \operatorname{atan2}(R_{21},R_{11})
$$

Directly from the quaternion:

$$
\phi =
\operatorname{atan2}
\left(
2(\epsilon_2\epsilon_3+\epsilon_1\eta),
1-2(\epsilon_1^2+\epsilon_2^2)
\right)
$$

$$
\theta =
-\sin^{-1}
\left(
2(\epsilon_1\epsilon_3-\epsilon_2\eta)
\right)
$$

$$
\psi =
\operatorname{atan2}
\left(
2(\epsilon_1\epsilon_2+\epsilon_3\eta),
1-2(\epsilon_2^2+\epsilon_3^2)
\right)
$$

## Navigation Coordinates

### Global Navigation Coordinates

ECEF coordinates are useful for satellite navigation and global maps, but operators usually need:

- longitude $l$,
- latitude $\mu$,
- ellipsoidal height $h$.

WGS-84 constants:

$$
r_e = 6{,}378{,}137\ \mathrm{m}
$$

$$
r_p = 6{,}356{,}752\ \mathrm{m}
$$

$$
\omega_e = 7.292115 \times 10^{-5}\ \mathrm{rad/s}
$$

$$
e = \sqrt{1-\left(\frac{r_p}{r_e}\right)^2} \approx 0.0818
$$

Parameter meanings:

- $r_e$: Earth's equatorial radius, also called the semi-major axis $a$ of the WGS-84 ellipsoid.
- $r_p$: Earth's polar radius, also called the semi-minor axis $b$.
- $\omega_e$: Earth's rotation rate relative to an inertial frame.
- $e$: first [eccentricity](https://en.wikipedia.org/wiki/Eccentricity_(mathematics)) of the ellipsoid. It measures how far WGS-84 is from a sphere.
- $l$: longitude, positive east from the Greenwich meridian.
- $\mu$: geodetic latitude, measured from the equatorial plane to the ellipsoid normal. This is not exactly the same as geocentric latitude.
- $h$: ellipsoidal height, positive outward along the ellipsoid normal.

The flattening can also be written as:

$$
f = \frac{r_e-r_p}{r_e}
$$

[Prime vertical](https://en.wikipedia.org/wiki/Earth_radius#Prime_vertical) radius of curvature:
$$
N =
\frac{r_e^2}
{\sqrt{r_e^2\cos^2\mu + r_p^2\sin^2\mu}}
$$

### ECEF and Geodetic Conversion

Given the vehicle or BODY-origin position vector expressed in ECEF:

$$
p_{eb}^e =
\begin{bmatrix}
x^e & y^e & z^e
\end{bmatrix}^T
$$

Here:

- $p$: position vector.
- subscript $eb$: BODY origin $b$ with respect to ECEF origin $e$.
- superscript $e$: vector components are expressed in the ECEF frame.
- $x^e,y^e,z^e$: ECEF Cartesian coordinates in meters.

Longitude:

$$
l = \operatorname{atan2}(y^e,x^e)
$$

Let:

$$
p = \sqrt{(x^e)^2 + (y^e)^2}
$$

Here $p$ is a scalar distance, not the position vector $p_{eb}^e$. It is the distance from the ECEF $z_e$ axis to the point projected onto the equatorial plane:

$$
p = \sqrt{(x^e)^2 + (y^e)^2}
$$

Initial latitude estimate:

$$
\tan(\mu_0) =
\frac{z^e}{p}(1-e^2)^{-1}
$$

Approximate $N$:

$$
N =
\frac{r_e^2}
{\sqrt{r_e^2\cos^2\mu_0 + r_p^2\sin^2\mu_0}}
$$

Approximate height:

$$
h = \frac{p}{\cos\mu_0} - N
$$

This is not the final exact height. It is the height estimate that corresponds to the current latitude estimate $\mu_0$ and curvature radius $N$. The algorithm then uses this $h$ to improve the latitude estimate, recomputes $N$ and $h$, and repeats until the change in latitude is small.

Improved latitude:

$$
\tan(\mu) =
\frac{z^e}{p}
\left(
1-e^2\frac{N}{N+h}
\right)^{-1}
$$

Iterate until $|\mu-\mu_0|$ is sufficiently small.

For given $(l,\mu,h)$, ECEF coordinates are:

$$
\begin{bmatrix}
x^e \\
y^e \\
z^e
\end{bmatrix}
=
\begin{bmatrix}
(N+h)\cos\mu\cos l \\
(N+h)\cos\mu\sin l \\
\left(\frac{r_p^2}{r_e^2}N+h\right)\sin\mu
\end{bmatrix}
$$

### Transformation Between ECEF and NED

The NED frame is the local tangent frame attached to a longitude-latitude point. Its orientation relative to ECEF is set by two angles:

- longitude $l$: where the tangent point is around Earth's rotation axis,
- latitude $\mu$: how far the tangent point is tilted away from the equator.

The slide form expresses the NED-to-ECEF rotation as two principal rotations:

$$
R_n^e(l,\mu) = R_{z,l}R_{y,-\mu-\pi/2}
$$

This maps vector components from NED to ECEF:

$$
x^e = R_n^e(l,\mu)x^n
$$

The first rotation, $R_{z,l}$, rotates around the ECEF $z_e$ axis to the correct longitude. The second rotation, $R_{y,-\mu-\pi/2}$, tilts the NED frame to the correct latitude.

The $-\pi/2$ term is the part that often feels mysterious. It is the baseline rotation needed even at zero latitude. At the equator and prime meridian, the local NED axes are:

$$
x_n \rightarrow +z_e,
\qquad
y_n \rightarrow +y_e,
\qquad
z_n \rightarrow -x_e
$$

So the NED frame is already rotated by $-\pi/2$ about the local/ECEF $y$ axis before any latitude is added. The extra $-\mu$ then tilts this baseline alignment from the equator to latitude $\mu$. The sign is negative because the right-hand positive rotation about $+y$ would move $x_n$ toward $-z_e$, while NED-to-ECEF needs the opposite direction under this convention.

Expanding the product gives:

$$
R_n^e(l,\mu) =
\begin{bmatrix}
-\sin\mu\cos l & -\sin l & -\cos\mu\cos l \\
-\sin\mu\sin l & \cos l & -\cos\mu\sin l \\
\cos\mu & 0 & -\sin\mu
\end{bmatrix}
$$

A useful sign check is the equator at the prime meridian:

$$
l=0,\qquad \mu=0
$$

At this point:

- North points along $+z_e$.
- East points along $+y_e$.
- Down points toward Earth's center, along $-x_e$.

Therefore:

$$
R_n^e(0,0) =
\begin{bmatrix}
0 & 0 & -1 \\
0 & 1 & 0 \\
1 & 0 & 0
\end{bmatrix}
$$

whose columns are exactly the ECEF coordinates of the North, East, and Down axes.

The ECEF-to-NED rotation is the transpose:

$$
R_e^n(l,\mu) = (R_n^e(l,\mu))^T
$$

so:

$$
x^n = R_e^n(l,\mu)x^e
$$

In full matrix form:

$$
R_e^n(l,\mu) =
\begin{bmatrix}
-\sin\mu\cos l & -\sin\mu\sin l & \cos\mu \\
-\sin l & \cos l & 0 \\
-\cos\mu\cos l & -\cos\mu\sin l & -\sin\mu
\end{bmatrix}
$$

The rows of $R_e^n$ are the local North, East, and Down unit vectors expressed in ECEF coordinates:

$$
\hat{n}^e =
\begin{bmatrix}
-\sin\mu\cos l & -\sin\mu\sin l & \cos\mu
\end{bmatrix}
$$

$$
\hat{e}^e =
\begin{bmatrix}
-\sin l & \cos l & 0
\end{bmatrix}
$$

$$
\hat{d}^e =
\begin{bmatrix}
-\cos\mu\cos l & -\cos\mu\sin l & -\sin\mu
\end{bmatrix}
$$

To use these rotations for position, first choose a local NED origin at geodetic coordinates:

$$
(l_0,\mu_0,h_0)
$$

and compute its ECEF position:

$$
p_{en}^e =
\begin{bmatrix}
x_n^e & y_n^e & z_n^e
\end{bmatrix}^T
$$

where the subscript $en$ means "NED origin with respect to ECEF origin." If the vehicle position in ECEF is:

$$
p_{eb}^e =
\begin{bmatrix}
x_b^e & y_b^e & z_b^e
\end{bmatrix}^T
$$

then the relative position from the NED origin to the vehicle is found by subtracting the origins first, then rotating:

$$
p_{nb}^n =
R_e^n(l_0,\mu_0)
\left(p_{eb}^e-p_{en}^e\right)
$$

The inverse position transformation is:

$$
p_{eb}^e = p_{en}^e + R_n^e(l_0,\mu_0)p_{nb}^n
$$

For attitude transformations, the same rotation composes with the vehicle attitude:

$$
R_b^n = R_e^n R_b^e
$$

and:

$$
R_b^e = R_n^e R_b^n
$$

This is the coordinate-frame version of the rule used earlier: adjacent superscript/subscript frames cancel in the middle. For example, $R_e^nR_b^e$ maps BODY coordinates to ECEF and then ECEF coordinates to NED, so the result maps BODY directly to NED.

For local navigation, $l_0$ and $\mu_0$ are fixed constants. For terrestrial navigation over larger distances, the local NED frame follows the vehicle, so $R_e^n(l,\mu)$ changes as longitude and latitude change.



### Local Flat-Earth Coordinates

> ch2.4

For small operating areas, the ECEF-to-NED transformation can be approximated by local flat-Earth coordinates. Choose a fixed NED tangent-plane origin $(l_0,\mu_0)$ with reference height $h_{ref}$.

The idea is to replace the curved ellipsoid by a tangent plane:

- changes in latitude become North displacement,
- changes in longitude become East displacement,
- changes in height become negative Down displacement.

Curvature radii:

$$
R_N =
\frac{r_e}
{\sqrt{1-e^2\sin^2\mu_0}}
$$

$$
R_M =
R_N
\frac{1-e^2}
{1-e^2\sin^2\mu_0}
$$

From local NED to longitude/latitude/height:

$$
\Delta l =
\frac{y^n}{(R_N+h_{ref})\cos\mu_0}
$$

$$
\Delta \mu =
\frac{x^n}{R_M+h_{ref}}
$$

$$
l = \operatorname{ssa}(l_0+\Delta l)
$$

$$
\mu = \operatorname{ssa}(\mu_0+\Delta \mu)
$$

$$
h = h_{ref} - z^n
$$

From longitude/latitude/height to local NED:

$$
\Delta l = l-l_0,
\qquad
\Delta \mu = \mu-\mu_0
$$

$$
x^n = \Delta\mu(R_M+h_{ref})
$$

$$
y^n = \Delta l(R_N+h_{ref})\cos\mu_0
$$

$$
z^n = h_{ref}-h
$$

`ssa` means smallest signed angle, normally confined to $[-\pi,\pi)$.

## Relative-Flow Geometry

### Course, Heading, and Crab

For horizontal motion with roll and pitch neglected:

$$
\dot{x}^n = u\cos\psi - v\sin\psi
$$

$$
\dot{y}^n = u\sin\psi + v\cos\psi
$$

The key relationship is:

$$
\chi = \psi + \beta_c
$$

where:

- $\chi$: course angle, the direction the vehicle is moving.
- $\psi$: heading/yaw angle, the direction the bow or $x_b$ axis points.
- $\beta_c$: horizontal crab angle.

Horizontal speed:

$$
U = \sqrt{u^2+v^2}
$$

Horizontal crab angle:

$$
\beta_c = \sin^{-1}\left(\frac{v}{U}\right)
$$

Amplitude-phase form:

$$
\dot{x}^n = U\cos(\psi+\beta_c) = U\cos\chi
$$

$$
\dot{y}^n = U\sin(\psi+\beta_c) = U\sin\chi
$$

Interpretation:

- Heading is measured by a compass.
- Course is measured by GNSS or underwater position reference systems.
- Crosswind, waves, and currents can make course differ from heading.

### Relative Velocity, AOA, and Sideslip

For ocean current:

$$
u_r = u-u_c,
\qquad
v_r = v-v_c,
\qquad
w_r = w-w_c
$$

Relative speed:

$$
U_r = \sqrt{u_r^2+v_r^2+w_r^2}
$$

Current speed:

$$
U_c = \sqrt{u_c^2+v_c^2+w_c^2}
$$

Angle of attack:

$$
\alpha = \operatorname{atan2}(w_r,u_r)
$$

Sideslip:

$$
\beta = \sin^{-1}\left(\frac{v_r}{U_r}\right)
$$

Horizontal crab and sideslip are different:

- $\beta_c$ is the angle between course and heading. It enters line-of-sight guidance laws.
- $\beta$ is the angle between the BODY $x_b$ axis and relative flow. It is used for hydrodynamic/aerodynamic coefficients.

They are equal only when there is no external horizontal flow:

$$
u_f = v_f = 0
$$

### 3-D Amplitude-Phase Velocity

The NED velocity can be described by speed $U$, course angle $\chi$, and flight-path angle $\gamma$:

$$
\chi = \operatorname{atan2}(\dot{y}^n,\dot{x}^n)
$$

$$
\gamma = \sin^{-1}\left(\frac{-\dot{z}^n}{U}\right)
$$

Horizontal speed:

$$
U_h = U\cos\gamma
$$

Velocity in NED:

$$
\dot{p}^n =
U
\begin{bmatrix}
\cos\gamma\cos\chi \\
\cos\gamma\sin\chi \\
-\sin\gamma
\end{bmatrix}
$$

Equivalently:

$$
\dot{p}^n =
\begin{bmatrix}
U_h\cos(\psi+\beta_c) \\
U_h\sin(\psi+\beta_c) \\
-U\sin(\theta-\alpha_c)
\end{bmatrix}
$$

with:

$$
\chi = \psi+\beta_c,
\qquad
\gamma = \theta-\alpha_c
$$

### BODY to FLOW Transformation

FLOW axes are found by rotating BODY axes until the FLOW $x$-axis is parallel to the relative flow.

Principal rotations:

1. Rotate by angle of attack $\alpha$ about $y$.
2. Rotate by $-\beta$ about $z$.

Intermediate stability axes:

$$
v_r^{stab} = R_{y,\alpha}v_r^b
$$

FLOW axes:

$$
v_r^{flow} = R_{z,-\beta}v_r^{stab}
$$

The BODY-to-FLOW rotation matrix is:

$$
R_b^{flow} = R_{z,-\beta}R_{y,\alpha}
$$

where:

$$
R_{y,\alpha} =
\begin{bmatrix}
\cos\alpha & 0 & \sin\alpha \\
0 & 1 & 0 \\
-\sin\alpha & 0 & \cos\alpha
\end{bmatrix}
$$

and:

$$
R_{z,-\beta} =
\begin{bmatrix}
\cos\beta & \sin\beta & 0 \\
-\sin\beta & \cos\beta & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

Thus:

$$
R_b^{flow} =
\begin{bmatrix}
\cos\beta\cos\alpha & \sin\beta & \cos\beta\sin\alpha \\
-\sin\beta\cos\alpha & \cos\beta & -\sin\beta\sin\alpha \\
-\sin\alpha & 0 & \cos\alpha
\end{bmatrix}
$$

Velocity transformation:

$$
v_r^{flow} = R_b^{flow}v_r^b
$$

and:

$$
v_r^b = (R_b^{flow})^T v_r^{flow}
$$

If:

$$
v_r^{flow} =
\begin{bmatrix}
U_r & 0 & 0
\end{bmatrix}^T
$$

then:

$$
u_r = U_r\cos\alpha\cos\beta
$$

$$
v_r = U_r\sin\beta
$$

$$
w_r = U_r\sin\alpha\cos\beta
$$

Therefore:

$$
\alpha = \tan^{-1}\left(\frac{w_r}{u_r}\right),
\qquad
\beta = \sin^{-1}\left(\frac{v_r}{U_r}\right)
$$

## Practical Reading Notes

- Use Euler angles when roll, pitch, and yaw interpretation matters and $\theta$ stays away from $\pm90^\circ$.
- Use quaternions for numerical integration when singularities and smooth attitude interpolation matter.
- Use NED for local navigation and vehicle equations.
- Use ECEF/geodetic coordinates for GNSS and global positioning.
- Use flat-Earth coordinates only for local-area operation.
- Keep $\beta_c$ and $\beta$ separate: crab is path-vs-heading; sideslip is relative-flow-vs-body.
