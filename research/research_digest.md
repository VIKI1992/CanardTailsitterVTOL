# Research Digest: 2 m Class Twin-Motor Canard Tailsitter (V3 Baseline)

## Scope
This digest supports a clean-sheet, 2 m class, twin-motor canard tailsitter baseline created from scratch for the `research/` and `spec/` package only. No geometry or implementation under `core/`, `src/`, `design/`, or `results/` was used as design input.

Legend:
- `L`: literature-backed statement
- `I`: engineering inference used to close the design where the literature does not directly specify a number

## Fixed Product Constraints
- `Constraint`: twin fixed motors, counter-rotating.
- `Constraint`: independently actuated elevons in propwash for hover and transition control.
- `Constraint`: symmetric all-moving canards for pitch authority and positive forward-flight lift.
- `Constraint`: control-augmented aircraft; strong open-loop passive stability not required.
- `Constraint`: smooth continuous fuselage loft, not a segmented placeholder.

## Source Digest

### 1. Kapsalis et al., Aerospace 10(3):264
Source: https://www.mdpi.com/2226-4310/10/3/264

- `L` The paper studies canard-wing optimization with VLM-based analysis and treats canard airfoil, size, incidence, and placement as first-order variables.
- `L` Canard airfoil choice and geometric placement materially affect drag and pitching-moment behavior.
- `L` Incidence and vertical placement are strong configuration drivers in canard-wing layouts.
- `I` For this baseline, the canard is sized and placed deliberately rather than left as a trim-only surface; this motivates a relatively large all-moving canard with low-to-moderate vertical offset and a small built-in positive incidence.

### 2. NASA NTRS 19790005842
Source: https://ntrs.nasa.gov/citations/19790005842

- `L` The referenced NASA work explicitly studies canard vertical location, size, and deflection as coupled interference variables in subsonic canard-wing configurations.
- `L` Vertical location is therefore not a cosmetic placement choice; it changes the aerodynamic interaction between foreplane and main wing.
- `I` This baseline keeps the canard near the wing reference plane instead of mounting it high above the fuselage, because low-to-moderate vertical separation is a safer starting point for a compact 2 m UAV that still needs strong pitch authority.

### 3. Spencer et al., NASA TM X-549 / NTRS 19980235628
Source: https://ntrs.nasa.gov/api/citations/19980235628/downloads/19980235628.pdf

- `L` The NASA study found that canard planform and area strongly affect low-speed longitudinal characteristics and trim lift.
- `L` The larger canard tested produced more trim lift than the smaller one over the tested angle-of-attack range.
- `L` A trapezoidal canard was more effective at low angle of attack; delta-like planforms only became more favorable at high deflection.
- `I` A trapezoidal all-moving canard is the right baseline here because the vehicle must spend meaningful time in forward flight and transition, not only at extreme foreplane deflection.
- `I` A canard area near 18 percent of main-wing area is a justified starting point for strong pitch control plus non-trivial positive cruise lift.

### 4. Bai et al., Agriculture 14(3):472
Source: https://www.mdpi.com/2077-0472/14/3/472

- `L` The paper highlights that propeller slipstream can increase lift and modify pitching moment, so low-speed propwash effects must be part of the conceptual design logic.
- `L` The authors report that short longitudinal separations can be harmful and that auxiliary lifting/control surfaces should not be placed with arbitrarily short moment arms.
- `L` The span of a free tail comparable to fuselage width at the separation location is recommended in that study's context.
- `I` For this tailsitter, those findings support two baseline choices: keep the canard moment arm substantial, and place hover/transition pitch-control surfaces inside propwash instead of relying on unblown surfaces alone.

### 5. TU Delft thesis repository item 78bc2287-dcbb-4bde-843b-3fa223bc0e53
Source: https://resolver.tudelft.nl/uuid:78bc2287-dcbb-4bde-843b-3fa223bc0e53

- `L` The repository item is directly concerned with propeller-slipstream effects on aircraft stability and control derivatives.
- `L` That is enough to treat slipstream as a primary modeling term rather than a late correction.
- `I` This baseline therefore defines powered and unpowered validation cases separately and does not assume one aerodynamic model is adequate for both.

### 6. de Wagter et al., Design and Control of a Tilt-Rotor Tailsitter Aircraft with Pivot-Free Transition
Source: https://research.tudelft.nl/en/publications/design-and-control-of-a-tilt-rotor-tailsitter-aircraft-with-pivot

- `L` The abstract reports a passive mechanism that redirects thrust and slipstream through transition and notes a significant increase in available rolling moment.
- `L` Transition control authority benefits when control effectors stay immersed in useful flow.
- `I` For a fixed-motor vehicle, the nearest equivalent is to keep the elevons in propwash over the usable hover and transition envelope and to blend control allocation with airspeed instead of switching abruptly.

### 7. Review of Designs and Flight Control Techniques of Hybrid and Convertible VTOL UAVs
Source: https://www.researchgate.net/publication/353966072_Review_of_Designs_and_Flight_Control_Techniques_of_Hybrid_and_Convertible_VTOL_UAVs

- `L` The review frames transition control and actuator blending/allocation as the central challenge for hybrid VTOL aircraft.
- `L` Over-actuated architectures benefit from explicit control allocation rather than ad hoc regime-specific mixes.
- `I` The authoritative spec therefore includes a virtual-control allocator from the start, with scheduled effectiveness for throttle differential, blown elevons, and the symmetric canard.

### 8. Actuators 13(6):225
Source: https://www.mdpi.com/2076-0825/13/6/225

- `L` This work presents a fixed-motor bi-rotor tailsitter concept using differential thrust and aerodynamic surfaces rather than rotor-tilt mechanisms.
- `L` The paper is directly relevant because it reinforces the simplicity and robustness benefits of avoiding mechanically tilting propulsion units.
- `I` Twin fixed counter-rotating propellers plus independent elevons is therefore a sound baseline architecture for hover and transition control.

## Cross-Source Design Implications
- `L` Canard placement, incidence, and size are first-order aerodynamic variables.
- `L` Propwash meaningfully changes low-speed lift, pitching moment, and available control authority.
- `L` Transition should be handled with blended or scheduled control allocation rather than a hard handoff.
- `I` The main wing should use moderate aspect ratio, moderate taper, and modest washout so the propwash-immersed elevons remain effective and tip stall is softened.
- `I` A relatively large canard is justified because the aircraft is intentionally control-augmented and the canard is required to carry positive lift in cruise.
- `I` A smooth fuselage is worth specifying now because bluff segmented placeholder bodies would contaminate the canard-wing interaction and invalidate later validation work.

## Baseline Decisions Taken From the Research

### Decisions treated as literature-supported
- Twin fixed motors and differential thrust retained as a credible tailsitter control architecture.
- Elevons are placed in propwash and remain part of the pitch/roll/yaw control set through transition.
- The canard is trapezoidal, all-moving, and large enough to provide both trim authority and positive forward-flight lift.
- Control allocation is scheduled continuously with airspeed or transition progress.
- Powered and unpowered aerodynamic validation are separated.

### Decisions that remain engineering inferences
- Exact wing span, wing area, fuselage loft station coordinates, mass target, and motor-prop size.
- Exact canard span, chord, pivot fraction, incidence, and deflection limits.
- Exact fin geometry and CG target.
- Exact numeric effectiveness matrix used in the initial control allocator.

## Resulting V3 Baseline Direction
- `I` Main wing: 2.00 m span, 0.62 m^2 area, moderate taper, low sweep, symmetric section, modest washout.
- `I` Canard: 0.72 m span, 0.112 m^2 area, all-moving, trapezoidal, pivot at 30 percent chord, rigged at +1.5 deg.
- `I` Propulsion: twin fixed 18x8 counter-rotating props, placed so both prop disks overlap the active elevon span.
- `I` Mass target: 4.4 kg MTOM with static thrust target at least 1.35 times weight.
- `I` Control model: weighted least-squares allocation onto `throttle_left`, `throttle_right`, `elevon_left`, `elevon_right`, and `canard_symmetric`.

## Open Risks to Validate Next
- `I` Whether the chosen symmetric main-wing airfoil gives adequate unpowered glide performance.
- `I` Whether canard downwash and upwash on the main wing are acceptable at high canard deflection during transition.
- `I` Whether the chosen motor y-location gives enough roll authority from both blown elevons and differential thrust without excess adverse yaw.
- `I` Whether the fuselage forebody locally disturbs canard effectiveness enough to require additional vertical offset.
