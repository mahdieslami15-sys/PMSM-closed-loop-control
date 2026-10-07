# FOC Development Report — 3-kW IPMSM Electric Motorcycle Drive

> **Development note:** The 3-kW IPMSM motorcycle has 10 poles, p = 5, and uses throttle-derived torque demand with no speed-control loop. A coordinated MTPA/field-weakening reference generator supplies $i_d^*$ and $i_q^*$. The actual model, remaining motor parameters, maps, gains, limits, and calibration were not supplied; the diagrams explain the control relationships. Test and performance results are documented separately.

The software is developed in MATLAB/Simulink for embedded microcontroller implementation. Rotor processing supplies the FOC angle/speed interface; external SVPWM receives the stationary-frame voltage commands.

## Contents

- [1. Drive equipment and motor](#1-drive-equipment-and-motor)
- [2. Electrical coordinates and transformations](#2-electrical-coordinates-and-transformations)
- [3. IPMSM model behind the current loops](#3-ipmsm-model-behind-the-current-loops)
- [4. How the current loops close](#4-how-the-current-loops-close)
- [5. Torque demand and current references](#5-torque-demand-and-current-references)
- [6. Current and voltage command limits](#6-current-and-voltage-command-limits)
- [7. Field weakening and MTPA](#7-field-weakening-and-mtpa)
- [8. Interfaces and Simulink structure](#8-interfaces-and-simulink-structure)
- [9. Complete electric-motorcycle torque FOC](#9-complete-electric-motorcycle-torque-foc)
- [Mathematical references](#mathematical-references)

## 1. Drive equipment and motor

The inverter converts DC power into phase excitation; electromagnetic torque drives the shaft and load. With $T_{\mathrm{shaft}}$ denoting delivered shaft torque, mechanical output power follows:

$$
P_{\mathrm{shaft}}=T_{\mathrm{shaft}}\omega_m
$$

![Drive equipment and connections](assets/figures/01-drive-equipment-and-foc.png)

**Figure 1.** Functional DC, A/B/C phase, and shaft connections are drawn directly on the equipment artwork. Three separate measured phase-current branches reach current acquisition; the rotor interface returns angle/speed information to FOC. The artwork is representative; the actual current-sensor implementation, equipment terminals, and rotor-position source are unspecified.

| Project input | Value |
|---|---|
| Application | 3-kW IPMSM electric motorcycle drive |
| Motor | 10 poles; p = 5 pole pairs |
| Development environment | MATLAB/Simulink |
| Intended target | Embedded microcontroller |

## 2. Electrical coordinates and transformations

### 2.1 Motor axes and electrical angle

The d-axis follows PM flux; q leads d by 90 electrical degrees. For raw mechanical angle, the five pole pairs multiply the angle and speed. Direction sign s = ±1 and electrical alignment offset $\theta_0$ complete the conversion:

$$
\theta_e=\operatorname{wrap}(s\,5\theta_m+\theta_0),\qquad
\omega_e=s\,5\omega_m
$$

With matched direction and zero offset, $\theta_e$ = 5θ_m and $\omega_e$ = 5ω_m: one mechanical revolution spans five electrical revolutions. An already aligned electrical-angle input passes directly to the transforms.

![IPMSM construction and reference axes](assets/figures/02-ipmsm-rotor-and-electrical-axes.png)

**Figure 2.** Illustrative ten-pole construction, electrical reference axes, and aligned 5× scaling occupy separate panels. The magnet leader identifies an embedded magnet. Slot/magnet geometry is illustrative; the confirmed project input is 10 poles, p = 5.

### 2.2 One vector, two coordinate descriptions

Changing the frame changes the components while preserving the physical vector and its norm:

$$
\mathbf i_s=i_\alpha\hat{\mathbf e}_\alpha+i_\beta\hat{\mathbf e}_\beta
=i_d\hat{\mathbf e}_d+i_q\hat{\mathbf e}_q,\qquad
i_\alpha^2+i_\beta^2=i_d^2+i_q^2
$$

![Stationary and rotating current components](assets/figures/03-stationary-and-rotating-frames.png)

**Figure 3.** The same purple current vector appears in both panels. Stationary αβ components and rotating dq components each add head-to-tail to reconstruct it.

| Convention | Definition |
|---|---|
| Clarke scaling | Amplitude-invariant 2/3; zero-sequence excluded |
| Axis orientation | α follows phase a; β leads α by +90°; q leads d by +90° |
| Electrical angle | $\theta_e$ is counterclockwise from α to d |
| References | Superscript * denotes a command/reference |
| Amplitudes | dq current/voltage amplitudes correspond to balanced phase peak |

### 2.3 Clarke: phase currents to stationary components

The weighted phase contributions produce the stationary current vector:

$$
\begin{bmatrix}i_\alpha\\i_\beta\end{bmatrix}
=\frac{2}{3}
\begin{bmatrix}1&-\tfrac12&-\tfrac12\\0&\tfrac{\sqrt3}{2}&-\tfrac{\sqrt3}{2}\end{bmatrix}
\begin{bmatrix}i_a\\i_b\\i_c\end{bmatrix}
$$

When $i_a$ + $i_b$ + $i_c$ = 0, this reduces to $i_\alpha$ = $i_a$ and $i_\beta$ = ($i_a$ + 2i_b)/√3. That shorter form depends on the zero-sum condition.

![Clarke transformation](assets/figures/04-clarke-transformation.png)

**Figure 4.** The normalized phase contributions add head-to-tail; the Clarke equations give the resulting αβ components. The reduced two-current equations are shown with their zero-phase-sum condition.

### 2.4 Park and inverse Park

Park rotates current coordinates into the rotor frame. Inverse Park rotates the commanded voltage back into the stationary frame using the same $\theta_e$:

$$
\begin{bmatrix}i_d\\i_q\end{bmatrix}
=\begin{bmatrix}\cos\theta_e&\sin\theta_e\\-\sin\theta_e&\cos\theta_e\end{bmatrix}
\begin{bmatrix}i_\alpha\\i_\beta\end{bmatrix},\qquad
\begin{bmatrix}v_\alpha^*\\v_\beta^*\end{bmatrix}
=\begin{bmatrix}\cos\theta_e&-\sin\theta_e\\\sin\theta_e&\cos\theta_e\end{bmatrix}
\begin{bmatrix}v_d^*\\v_q^*\end{bmatrix}
$$

Both rotations preserve vector magnitude. This connects rotor-frame current feedback to stationary-frame inverter commands.

![Park and inverse Park](assets/figures/05-park-and-inverse-park.png)

**Figure 5.** Separate panels show current-feedback Park, αβ → dq, and voltage-command inverse Park, dq → αβ. The physical vector and norm remain unchanged.

### 2.5 Why sinusoidal currents become steady dq components

For an ideal balanced pure-q example with amplitude I and synchronous $\theta_e$:

$$
i_a=I\cos(\theta_e+\tfrac{\pi}{2}),\qquad
i_\alpha=-I\sin\theta_e,\qquad i_\beta=I\cos\theta_e
\quad\Longrightarrow\quad i_d=0,\quad i_q=I
$$

![Analytical abc alpha-beta and dq waveforms](assets/figures/06-balanced-abc-to-dq.png)

**Figure 6.** Aligned analytical waveforms show I = 1, $i_d$ = 0, and $i_q$ = 1. The 35° marker connects the waveforms to their frame snapshots.

### Animated example A1 — Synchronous current transformation

![Balanced phase currents and synchronous dq projections](assets/animations/abc_dq_projection.gif)

The same balanced current vector becomes constant $i_d$ and $i_q$ when the Park angle rotates synchronously. This normalized illustration follows one electrical cycle. For the confirmed five pole pairs, one aligned mechanical revolution spans five electrical revolutions. The values illustrate geometry rather than measured motor response.

### Animated example A2 — Electrical-angle alignment

![Effect of electrical-angle offset on measured dq currents](assets/animations/electrical_angle_alignment.gif)

An angle offset rotates the measurement frame and mixes its d- and q-components without changing the physical current vector. At a fixed offset, the measured dq components remain constant. This analytical illustration does not model an observer, sensor or closed-loop drive response.

Animation A1 explores the same geometry with mixed d/q components. A2 varies angle alignment while retaining the physical current vector. Both use electrical angle; for p = 5, five electrical revolutions correspond to one mechanical revolution under aligned rotation.

## 3. IPMSM model behind the current loops

The linear-parameter synchronous model links current to flux:

$$
\psi_d=L_di_d+\psi_f,\qquad\psi_q=L_qi_q
$$

The applied voltages must supply resistance, current dynamics, and speed-dependent terms:

$$
v_d=R_si_d+L_d\frac{di_d}{dt}-\omega_eL_qi_q
$$

$$
v_q=R_si_q+L_q\frac{di_q}{dt}+\omega_e(L_di_d+\psi_f)
$$

Torque combines PM and saliency contributions; mechanical acceleration follows the torque balance:

$$
T_e=\underbrace{\frac32p\psi_fi_q}_{T_{PM}}
+\underbrace{\frac32p(L_d-L_q)i_di_q}_{T_{rel}},\qquad p=5
$$

$$
J\frac{d\omega_m}{dt}=T_e-T_L-B\omega_m
$$

![IPMSM flux voltage and torque terms](assets/figures/07-ipmsm-model-and-torque.png)

**Figure 7.** Flux linkages, signed voltage terms, and torque components are separated. The q-axis speed terms distinguish cross-coupling from PM back-EMF. p = 5 is confirmed; $R_s$, $L_d$, $L_q$, $\psi_f$, J, B, and the inductance ordering are unspecified.

## 4. How the current loops close

### 4.1 Overall control architecture

Each current controller compares a reference with the corresponding measured component. Its PI output is a voltage request:

$$
\begin{aligned}
e_d&=i_d^*-i_d,&
u_d&=K_{pd}e_d+K_{id}\int e_d\,dt\\
e_q&=i_q^*-i_q,&
u_q&=K_{pq}e_q+K_{iq}\int e_q\,dt
\end{aligned}
$$

The motorcycle's throttle interface requests electrical motor torque. One coordinated reference function uses the torque request, electrical speed, available voltage, and machine bounds:

$$
\begin{aligned}
T_{\mathrm{req}}^*&=f_{\mathrm{throttle}}(a_{\mathrm{th}},\text{vehicle state})\\
(i_d^*,i_q^*,T_{\mathrm{acc}}^*)&=
\mathcal R(T_{\mathrm{req}}^*,\omega_e,V_{\mathrm{dc}},V_{\mathrm{req}},\text{model and bounds})
\end{aligned}
$$

$T_{\mathrm{acc}}^*$ is the feasible accepted electromagnetic torque; it can be lower than the rider request. The function R groups MTPA, voltage-feedback field weakening, and joint feasibility. It returns both current references. The two complete voltage requests share one vector limiter and inverse Park. Phase-current feedback returns through Clarke/Park as $i_d$ and $i_q$; $\theta_e$ aligns both transforms. Electrical speed feeds operating-point calculations and optional compensation. There is no speed setpoint or speed PI.

![Overall FOC control architecture](assets/figures/08-torque-command-foc-architecture.png)

**Figure 8.** Torque demand enters one coordinated current-reference block; its outputs feed the matching d/q current errors. The separated lower feedback paths distinguish measured currents, electrical angle, electrical speed, and available voltage. One shared dq limiter precedes inverse Park and external SVPWM.

### 4.2 d-axis loop

Rearranging the motor equation shows how d-voltage controls d-current. The reference-minus-feedback error gives the required negative-feedback connection:

$$
\begin{aligned}
L_d\frac{di_d}{dt}+R_si_d&=v_d+\omega_eL_qi_q\\
e_d=i_d^*-i_d,\qquad
u_d&=K_{pd}e_d+K_{id}\int e_d\,dt
\end{aligned}
$$

If $i_d$ is below its reference, $e_d$ is positive and the PI requests a higher d-voltage under this sign convention.

![d-axis current loop](assets/figures/09-d-axis-current-loop.png)

**Figure 9.** Measured $i_d$ returns to the negative d-error input. The companion q-voltage joins the same limiter and two-axis actuator; acquisition and Clarke/Park close the current path.

### 4.3 q-axis loop

The q-voltage must also overcome the speed-dependent flux term:

$$
\begin{aligned}
L_q\frac{di_q}{dt}+R_si_q&=v_q-\omega_e(L_di_d+\psi_f)\\
e_q=i_q^*-i_q,\qquad
u_q&=K_{pq}e_q+K_{iq}\int e_q\,dt
\end{aligned}
$$

The PI acts on q-current error; the plant's q-current response returns to the matching negative input.

![q-axis current loop](assets/figures/10-q-axis-current-loop.png)

**Figure 10.** Measured $i_q$ closes the q-error channel. Both axes use the same voltage limiter, inverse Park, SVPWM, inverter, and motor.

### 4.4 Optional decoupling feedforward

For the stated model convention, complete signed feedforward terms are added to the PI outputs:

$$
v_{d,\mathrm{raw}}^*=u_d-\omega_eL_qi_q,\qquad
v_{q,\mathrm{raw}}^*=u_q+\omega_e(L_di_d+\psi_f)
$$

With exact parameters and no voltage saturation, substitution cancels the modeled speed terms:

$$
L_d\frac{di_d}{dt}+R_si_d=u_d,\qquad
L_q\frac{di_q}{dt}+R_si_q=u_q
$$

![Signed cross-coupling feedforward](assets/figures/11-decoupling-and-feedforward.png)

**Figure 11.** Negative d-axis coupling and positive q-axis coupling/back-EMF enter the voltage requests as complete signed terms. The common limiter acts on both requests before inverse Park. Feedforward is an optional structure.

## 5. Torque demand and current references

### 5.1 Throttle torque command and inner current loops

The torque interface receives the conditioned throttle request. Current tracking produces electromagnetic torque through the IPMSM model; mechanical speed follows the load dynamics:

$$
\begin{aligned}
T_e&=\frac32p\,i_q[\psi_f+(L_d-L_q)i_d]\\
J\frac{d\omega_m}{dt}&=T_e-T_L-B\omega_m,\qquad p=5
\end{aligned}
$$

The requested torque is not a speed target. Measured speed remains useful for voltage feasibility and feedforward. The throttle map, conditioning and any wheel-to-motor torque conversion are unspecified. If the request is defined at the wheel, gearing and efficiency must be handled before this electrical motor-torque interface.

![Torque command and inner current feedback](assets/figures/12-throttle-torque-and-current-loops.png)

**Figure 12.** Throttle-derived torque reaches the reference coordinator; measured dq currents close the electrical loops. Motor/load dynamics determine speed, which feeds operating-point calculations rather than a speed-error controller.

### 5.2 Torque-consistent reference allocation

While voltage headroom is adequate, MTPA selects a minimum-current baseline for the accepted torque. Field weakening can change its d-component. The q-reference must then be recalculated for that selected d-current:

$$
\begin{aligned}
i_d^*&=i_{d,0}^*+\Delta i_{d,\mathrm{FW}},\qquad \Delta i_{d,\mathrm{FW}}\le0\\
i_q^*&=\frac{T_{\mathrm{acc}}^*}
{\frac32p[\psi_f+(L_d-L_q)i_d^*]},\qquad p=5
\end{aligned}
$$

This mapping assumes a positive torque coefficient bounded away from zero. The coordinator checks both current and voltage feasibility, plus machine bounds. If the pair cannot deliver the rider demand, it reduces $T_{\mathrm{acc}}^*$ and resolves the pair. An independent q-current clip would change torque and must be included in that accepted-torque calculation. Section 7 develops the MTPA and FW relationships.

![Current-reference generation](assets/figures/13-coordinated-current-references.png)

**Figure 13.** One torque-to-current coordinator combines MTPA, bounded FW correction, and joint feasible torque selection. Both selected current references correspond to $T_{\mathrm{acc}}^*$, which is identified explicitly when constraints reduce the request.

## 6. Current and voltage command limits

The current-reference circle and voltage-command circle constrain different vectors:

$$
\sqrt{(i_d^*)^2+(i_q^*)^2}\le I_{\max},\qquad
\sqrt{(v_d^*)^2+(v_q^*)^2}\le V_{\mathrm{lim}}
$$

These amplitudes use balanced phase-peak scaling. $V_{\mathrm{lim}}$ depends on the DC link, modulation convention, and available margin. Animation A3 denotes its normalized illustrative bound by Vmax.

![Current and voltage vector limits](assets/figures/14-current-and-voltage-limits.png)

**Figure 14.** Separate normalized planes show current-reference and voltage-command constraints. Radial projection is an illustrative policy that preserves direction and maps a zero request to zero. The motorcycle coordinator instead selects a jointly feasible current pair for accepted torque; current-only projection is not a same-torque allocation. Animation A3 extends the voltage-command geometry.

### Animated example A3 — Shared voltage-vector limitation

![Direction-preserving radial voltage-vector limitation](assets/animations/voltage_vector_limit.gif)

A sample radial rescaling limits command magnitude while preserving the voltage-vector direction and component ratio. The illustration uses normalized $V_{\max}=1$. It demonstrates limiter geometry; the physical voltage bound and selected project limiter policy require their own definition.

Voltage limitation bounds the applied command; it cannot by itself make an infeasible current reference achievable. Field weakening instead changes the current reference to reduce the required voltage. Saturation handling and anti-windup depend on the controller realization.

## 7. Field weakening and MTPA

### 7.1 Why negative d-current creates voltage headroom

At steady current, the derivative terms vanish. Retaining stator resistance gives:

$$
v_{d,ss}=R_si_d-\omega_eL_qi_q,\qquad
v_{q,ss}=R_si_q+\omega_e(L_di_d+\psi_f)
$$

A current pair is voltage-feasible only if:

$$
(R_si_d-\omega_eL_qi_q)^2+
\bigl[R_si_q+\omega_e(L_di_d+\psi_f)\bigr]^2
\le V_{\mathrm{lim}}^2
$$

For the geometric approximation that neglects $R_s$ and assumes |$\omega_e$| > 0:

$$
(L_di_d+\psi_f)^2+(L_qi_q)^2
\le \left(\frac{V_{\mathrm{lim}}}{|\omega_e|}\right)^2
$$

This is a voltage ellipse in the **current $i_d$/$i_q$ plane**, centered at (−$\psi_f$/$L_d$, 0). It shrinks as speed increases. Moving $i_d$ below zero toward that center reduces the net d-axis flux $\psi_d$ = $\psi_f$ + $L_d$ $i_d$; the PM parameter $\psi_f$ remains fixed in this model.

For p = 5 and a selected current pair, the same resistance-neglect approximation gives the voltage-limited mechanical-speed bound, where the denominator is nonzero:

$$
|\omega_m|\le
\frac{V_{\mathrm{lim}}}
{5\sqrt{(L_di_d+\psi_f)^2+(L_qi_q)^2}}
$$

At fixed $i_q$, moving $i_d$ from zero into −$\psi_f$/$L_d$ < $i_d$ < 0 reduces the denominator and can increase this bound. The comparison uses more total current; current capacity and torque feasibility still apply.

Negative $i_d$ uses part of the current allowance, leaving less room for q-current:

$$
i_d^2+i_q^2\le I_{\max}^2,\qquad
|i_q|\le \sqrt{\max(I_{\max}^2-i_d^2,\,0)}
$$

![Current-plane voltage ellipse and field weakening](assets/figures/15-field-weakening-geometry.png)

**Figure 15.** The current circle remains fixed while the voltage ellipse shrinks with electrical speed. The zero-d example A is feasible at the lower speed and voltage-infeasible at the higher speed. Negative-d/reduced-q example B lies in the higher-speed overlap. Parameters are illustrative and normalized; B demonstrates feasibility rather than equal torque or optimality. This is FW constraint geometry, not a computed MTPA locus.

Negative $i_d$ can extend the voltage-feasible speed range, subject to the current circle and the remaining machine/controller bounds. No project base speed or speed-extension value is available.

### 7.2 A voltage-feedback field-weakening loop

Use the magnitude of the **requested** dq voltage before the common limiter. Measuring only the already limited output would hide excess demand:

$$
V_{\mathrm{req}}=
\sqrt{(v_{d,\mathrm{raw}}^*)^2+(v_{q,\mathrm{raw}}^*)^2},\qquad
e_V=V_{\mathrm{FW}}-V_{\mathrm{req}},\qquad
0<V_{\mathrm{FW}}\le V_{\mathrm{lim}}
$$

$V_{\mathrm{FW}}$ is the chosen voltage target, with margin relative to the available vector limit $V_{\mathrm{lim}}$. It follows available DC-link voltage when that availability changes. Use identical voltage units/scaling for $V_{\mathrm{req}}$ and $V_{\mathrm{FW}}$. Starting from the MTPA d-reference $i_{d,0}^*$, excess demand gives negative $e_V$ and a nonpositive correction:

$$
\begin{aligned}
\Delta i_{d,\mathrm{raw}}&=K_{p,\mathrm{FW}}e_V+x_{\mathrm{FW}},
&K_{p,\mathrm{FW}},K_{i,\mathrm{FW}}&>0\\
\Delta i_{d,\mathrm{FW}}&=\operatorname{clip}
(\Delta i_{d,\mathrm{raw}},\,i_{d,\min}-i_{d,0}^*,\,0)\\
i_d^*&=i_{d,0}^*+\Delta i_{d,\mathrm{FW}}
\end{aligned}
$$

The MTPA baseline must itself satisfy d/current/machine bounds; its lower correction bound is nonpositive. FW state handling tracks the actually selected correction, including changes made by the joint coordinator. One positive-gain back-calculation realization is:

$$
\dot x_{\mathrm{FW}}=K_{i,\mathrm{FW}}e_V+
K_{aw,\mathrm{FW}}\bigl[(i_d^*-i_{d,0}^*)-\Delta i_{d,\mathrm{raw}}\bigr],
\qquad K_{aw,\mathrm{FW}}>0
$$

During inactive FW, track zero correction so $i_d^*$ follows MTPA. As voltage margin returns, release the correction toward zero with bumpless state handling. The selected d-current returns to the MTPA baseline, which need not be zero. This feedback sign applies locally where a more-negative d-reference along the coordinated accepted-torque pair lowers required voltage.

For each selected d-reference, the coordinator recalculates q-current using the accepted torque and checks the joint bounds:

$$
\begin{aligned}
i_q^*&=\frac{T_{\mathrm{acc}}^*}{\frac32p[\psi_f+(L_d-L_q)i_d^*]}\\
(i_d^*)^2+(i_q^*)^2&\le I_{\max}^2,\qquad
v_{d,ss}^2+v_{q,ss}^2\le V_{\mathrm{FW}}^2
\end{aligned}
$$

Here $v_{d,\mathrm{ss}}$ and $v_{q,\mathrm{ss}}$ are the resistance-retained steady-state voltages from Section 7.1. The d/machine bounds and valid nonzero torque coefficient also apply. Reduce accepted torque if the pair is infeasible, then resolve the pair. FW correction tracking and current-PI voltage-saturation handling serve distinct limits; there is no speed-PI state.

![Voltage-feedback field-weakening control](assets/figures/16-voltage-feedback-field-weakening.png)

**Figure 16.** Complete requested voltage before the joint limiter drives the FW correction relative to MTPA. Selected-correction tracking handles clamping and release. Torque-consistent q recomputation and joint feasibility determine the final current pair and accepted torque.

Steady-state feasibility does not guarantee every reference transient can be produced; voltage feedback and the command limiter handle the instantaneous demand. The loop is a conceptual local realization, not a global stability proof or an MTPV optimizer. $V_{\mathrm{FW}}$, d-bounds, gains, and state handling remain symbolic design choices. If the joint feasible set is empty, an infeasibility/protection response is required; the project's behavior is unspecified.

### 7.3 MTPA serves a different objective

Maximum torque per ampere (MTPA) selects a current pair for a specified accepted torque $T_{\mathrm{acc}}^*$. In the declared linear model:

$$
(i_{d,0}^*,i_{q,0}^*)
=\underset{i_d,i_q}{\operatorname{arg\,min}}\;(i_d^2+i_q^2)
\quad\text{subject to}\quad
\frac32p\bigl[\psi_fi_q+(L_d-L_q)i_di_q\bigr]=T_{\mathrm{acc}}^*
$$

For an interior solution, write Δ = $L_d$ − $L_q$. Differentiating the current norm along a constant-torque curve gives:

$$
i_d(\psi_f+\Delta i_d)=\Delta i_q^2
$$

MTPA minimizes current for accepted torque while voltage is not the active boundary, subject to applicable current/d-axis/machine bounds. If $\psi_f$ > 0 and $L_q$ > $L_d$, the motoring solution can have negative $i_d$ before FW; this motor's inductance ordering is unknown. A zero-d reference is a simplified teaching alternative, not the actual MTPA baseline. Saturating machines can require characterized parameter/reference maps instead of constant inductances.

| Strategy | Objective | Input/reference contract | d-current interpretation |
|---|---|---|---|
| MTPA baseline | Minimize current magnitude for accepted torque | $T_{\mathrm{acc}}^*$, motor model/maps, and machine bounds | May already be negative below FW; saliency determines the pair |
| Field weakening | Meet the voltage target as speed rises | MTPA baseline, $V_{\mathrm{req}}$, $V_{\mathrm{FW}}$, speed, and joint bounds | Adds a nonpositive correction; q is recomputed for accepted torque |
| Simplified zero-d example | Illustrate PM torque with adequate voltage headroom | Explicit torque-to-q mapping | $i_d^*$ = 0; not assumed to be IPMSM MTPA |

In the motorcycle, MTPA and FW operate within one torque-to-current coordinator. The accepted torque is explicit when constraints intervene. MTPV can be an optional high-speed extension where the feasible operating region requires it; its implementation is not confirmed here.

## 8. Interfaces and Simulink structure

### 8.1 Rotor and SVPWM interfaces

Raw mechanical position uses the confirmed five pole pairs; an already calibrated $\theta_e$ passes directly to Park and inverse Park:

$$
\theta_e=\operatorname{wrap}(s\,5\theta_m+\theta_0),\qquad
\omega_e=s\,5\omega_m,\qquad s\in\{-1,+1\}
$$

The direction, offset, and wrap convention belong to the input contract. Do not multiply an already aligned $\theta_e$ by 5 again.

| Signal | Role | Unit / meaning |
|---|---|---|
| $i_a$, $i_b$, $i_c$ | Current feedback input | A; phase currents |
| $\theta_e$ | Park and inverse Park input | Electrical rad; aligned angle |
| $a_{\mathrm{th}}$ | Throttle-map input | Rider command; actual electrical range/map unspecified |
| $T_{\mathrm{req}}^*$ | Coordinator torque demand | N·m; electrical motor-torque definition |
| $T_{\mathrm{acc}}^*$ | Accepted torque / limit status | N·m; model torque of the selected feasible current pair |
| $\omega_m$ | Rotor processing / operating point | Mechanical rad/s; no speed-error loop |
| $\omega_e$ | Speed-dependent model/feedforward input | Electrical rad/s |
| $V_{\mathrm{req}}$ | FW feedback input | V; norm of complete dq request before limitation |
| $i_d^*$, $i_q^*$ | Current-loop references | A; phase-peak dq scaling |
| $v_d^*$, $v_q^*$ | Limited internal voltage command | V; rotor-frame vector |
| $v_\alpha^*$, $v_\beta^*$ | Output to external SVPWM | Stationary-frame voltage; physical or explicitly normalized scaling |
| $V_{\mathrm{dc}}$, bounds | Voltage/current constraint inputs | Defined by the selected interface and reference policy |

![Rotor-position and SVPWM interfaces](assets/figures/17-rotor-and-svpwm-interfaces.png)

**Figure 17.** Torque demand reaches the reference coordinator. Electrical angle reaches both transforms; electrical speed feeds feasibility and optional compensation. FOC supplies stationary voltage references to external SVPWM, which supplies inverter PWM/duty commands.

### 8.2 Simulink functional structure

Using C for the amplitude-invariant Clarke matrix and P($\theta_e$) for Park, the two transform paths are:

$$
\mathbf i_{dq}=P(\theta_e)C\mathbf i_{abc},\qquad
\mathbf v_{\alpha\beta}^*=P^{-1}(\theta_e)\mathbf v_{dq}^*
$$

The torque-to-current coordinator, PI channels, optional feedforward, common voltage limitation, and inverse Park form the forward path. Phase currents close the feedback path. Rotor processing and available DC-link voltage supply the operating-point inputs.

![Conceptual Simulink FOC structure](assets/figures/18-simulink-subsystems.png)

**Figure 18.** The conceptual structure groups the MTPA/FW/feasibility coordinator, current control, and voltage output. Phase currents and electrical angle feed the feedback transforms; speed and voltage bounds enter reference handling. Stationary voltage commands point outward to SVPWM.

## 9. Complete electric-motorcycle torque FOC

### 9.1 Admit torque and select a feasible current pair

The rider requests torque through the throttle, rather than a speed setpoint. The coordinator admits torque that the current/voltage/machine bounds can support. For positive motoring demand, this contract can be written symbolically:

$$
\begin{aligned}
T_{\mathrm{acc}}^*&=\max\bigl\{T:\ 0\le T\le T_{\mathrm{req}}^*,\
\exists(i_d,i_q)\in\mathcal F,\ T_e(i_d,i_q)=T\bigr\}\\
\mathcal F&=\bigl\{(i_d,i_q):\ i_d^2+i_q^2\le I_{\max}^2,\
v_{d,ss}^2+v_{q,ss}^2\le V_{\mathrm{FW}}^2,\
i_{d,\min}\le i_d\le i_{d,\max},\ \text{machine bounds}\bigr\}
\end{aligned}
$$

F depends on electrical speed and available voltage. MTPA supplies the inactive-FW baseline; the nonpositive FW correction changes d-current when voltage becomes restrictive. q-current is recalculated from selected d-current and accepted torque. A calibrated lookup or validated constrained allocation can implement this contract; no particular embedded solver is assumed. Regenerative torque admission needs separate battery/vehicle policies, which are not supplied.

### 9.2 Close the current paths and handle voltage saturation

The complete raw dq request is tapped for $V_{\mathrm{req}}$ before limitation. A common direction-preserving vector limiter is one illustrative realization:

$$
\begin{aligned}
\gamma&=\begin{cases}
1,&\|\mathbf v_{dq,\mathrm{raw}}^*\|=0\\
\min\bigl(1,V_{\mathrm{lim}}/\|\mathbf v_{dq,\mathrm{raw}}^*\|\bigr),&\text{otherwise}
\end{cases}\\
\mathbf v_{dq}^*&=\gamma\mathbf v_{dq,\mathrm{raw}}^*
\end{aligned}
$$

The limited-minus-raw voltage residual returns to the matching PI states. For positive back-calculation gains:

$$
\begin{aligned}
\dot x_d&=K_{id}e_d+K_{aw,d}(v_d^*-v_{d,\mathrm{raw}}^*)\\
\dot x_q&=K_{iq}e_q+K_{aw,q}(v_q^*-v_{q,\mathrm{raw}}^*),\qquad
K_{aw,d},K_{aw,q}>0
\end{aligned}
$$

FW state tracking uses its selected current correction, while these current PI states track voltage saturation. Three acquired phase currents return through Clarke/Park to the matching negative d/q error inputs. Electrical angle reaches Park and inverse Park; electrical speed reaches feasibility and optional feedforward. Measured DC-link voltage supplies available-voltage limits. The actuator path continues through inverse Park, external SVPWM, inverter, motor, and drivetrain/load.

![Complete electric-motorcycle torque FOC](assets/figures/19-motorcycle-torque-foc.png)

**Figure 19.** Complete throttle-to-torque FOC for the electric motorcycle. The coordinator combines MTPA, voltage-feedback FW, and joint feasibility, supplying both current references. The pre-limiter voltage norm feeds FW; the voltage residual feeds the matching current PI anti-windup channels. Current and rotor feedback close the corresponding paths. $T_{\mathrm{acc}}^*$ is accepted model torque/limit status, not a measured-torque feedback loop. There is no speed PI, and inactive/released FW returns to MTPA rather than forcing $i_d^*$ = 0.

## Mathematical references

- [Clarke Transform — amplitude-invariant equations](https://www.mathworks.com/help/mcb/ref/clarketransform.html)
- [Park Transform — axis alignment and transformation matrix](https://www.mathworks.com/help/mcb/ref/parktransform.html)
- [PMSM HDL — amplitude-invariant motor equations](https://www.mathworks.com/help/mcb/ref/pmsmhdl.html)
- [PMSM FeedForward Control — compensation signs](https://www.mathworks.com/help/mcb/ref/pmsmfeedforwardcontrol.html)
- [PMSM Current Controller — PI control and voltage limitation](https://www.mathworks.com/help/simscape-electrical/ref/pmsmcurrentcontroller.html)
- [Field-Weakening Control with MTPA — operating constraints and reference strategies](https://www.mathworks.com/help/mcb/gs/field-weakening-control-mtpa-pmsm.html)
- [MTPA Control Reference — torque demand and current/voltage constraints](https://www.mathworks.com/help/mcb/ref/mtpacontrolreference.html)
- [Interior PM Controller — torque control and coordinated current references](https://www.mathworks.com/help/autoblks/ref/interiorpmcontroller.html)
- [TI SPRACF3 — voltage-feedback field weakening and MTPA reference contracts](https://www.ti.com/lit/an/spracf3/spracf3.pdf)
