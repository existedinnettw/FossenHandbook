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
- $v_{nb}^b$: linear velocity of BODY origin relative to NED origin, expressed in BODY.
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

- **CO**: coordinate origin.
- **CF**: center of flotation, the centroid of the calm-water waterplane area $A_{wp}$.

For small-angle linear theory, the vessel rolls and pitches about CF.

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

Euler's rotation theorem says that any rotation can be described by angle $\omega$ about a unit axis $\epsilon$:

$$
R_{\omega,\epsilon}
= I_3 + \sin(\omega)S(\epsilon)
+ (1-\cos(\omega))S^2(\epsilon)
$$

This is Rodrigues' rotation formula.

### Euler-Angle Attitude Representation

Marine craft and aircraft usually use the Tait-Bryan ZYX sequence:

1. Yaw $\psi$ about $z$.
2. Pitch $\theta$ about $y$.
3. Roll $\phi$ about $x$.

The Euler angle vector is:

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

Using $c(\cdot)=\cos(\cdot)$ and $s(\cdot)=\sin(\cdot)$:

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

Euler angles are intuitive but singular at:

$$
\theta = \pm 90^\circ
$$

Discrete position update with sampling time $h$:

$$
p_{nb}^n[k+1] = p_{nb}^n[k] + hR_b^n[k]v_{nb}^b[k]
$$

The slides recommend higher-order integration, such as RK4, for better numerical behavior.

### Quaternion Attitude Representation

Unit quaternions avoid Euler-angle singularity by using four parameters:

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

The quaternion rotation matrix is:

$$
R(q_b^n) = I_3 + 2\eta S(\epsilon) + 2S^2(\epsilon)
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

where

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

ECEF coordinates are useful for satellite navigation, but operators usually need:

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

Prime vertical radius of curvature:

$$
N =
\frac{r_e^2}
{\sqrt{r_e^2\cos^2\mu + r_p^2\sin^2\mu}}
$$

### ECEF and Geodetic Conversion

Given:

$$
p_{eb}^e =
\begin{bmatrix}
x^e & y^e & z^e
\end{bmatrix}^T
$$

Longitude:

$$
l = \operatorname{atan2}(y^e,x^e)
$$

Let:

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

Height:

$$
h = \frac{p}{\cos\mu_0} - N
$$

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

### Local Flat-Earth Coordinates

For local navigation, choose a fixed NED tangent-plane origin $(l_0,\mu_0)$ with reference height $h_{ref}$.

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
