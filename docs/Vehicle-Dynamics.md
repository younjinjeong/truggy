# Vehicle Dynamics

Physical parameters and dynamics model for the 1/8 scale TLR Truggy.

## Physical Parameters

| Parameter | TruggyAD (1/8) | AutoRally (1/5) | Units |
|-----------|----------------|-----------------|-------|
| Scale | 1:8 | 1:5 | — |
| Wheelbase | ~0.35 | 0.57 | m |
| Track width | ~0.30 | 0.40 | m |
| Tire diameter | ~0.12 | 0.19 | m |
| Chassis mass | ~4-5 | 20.5 | kg |
| Total mass (w/ electronics) | ~5-6 | ~22 | kg |

**Note:** Exact measurements needed after hardware assembly. Update this document with measured values.

## Coordinate Frame

```
        +X (forward)
         ^
         |
         |
+Y <-----O      (right-hand rule, Z points up)
         
         Z (up, out of page)
```

- **Body frame:** Origin at rear axle center, X forward, Y left, Z up
- **World frame:** Fixed ENU (East-North-Up), origin at start position

## Bicycle Model (Simplified)

The MPPI dynamics model uses a neural network, but the underlying physics follows a bicycle model:

```
dx/dt     = cos(yaw) * u_x - sin(yaw) * u_y    (world X velocity)
dy/dt     = sin(yaw) * u_x + cos(yaw) * u_y    (world Y velocity)
dyaw/dt   = yaw_rate                             (heading change)
du_x/dt   = f_nn(u_x, u_y, yaw_rate, steer, throttle)  (learned)
du_y/dt   = f_nn(...)                                     (learned)
dyaw_rate/dt = f_nn(...)                                  (learned)
droll/dt  = f_nn(...)                                     (learned)
```

The kinematics (first 3 equations) are analytically computed. The dynamics (last 4) are learned by the neural network from driving data.

## Key Dynamic Behaviors

### Understeer / Oversteer

At low speeds (< 3 m/s), the truggy behaves predictably. At higher speeds:
- **Understeer:** Front tires lose grip first, car goes wider than intended
- **Oversteer:** Rear tires lose grip first, car rotates more than intended
- The MPPI cost function penalizes excessive slip angle to prevent loss of control

### Slip Angle

```
slip_angle = atan2(u_y, u_x)
```

- Small slip angle (< 0.3 rad): tire operates in linear region, predictable
- Large slip angle (> 1.0 rad): tire saturated, risk of spin
- `max_slip_angle` parameter (default 1.25 rad) triggers penalty

### Weight Transfer

During braking/acceleration and cornering, weight shifts:
- Braking → more weight on front → more front grip
- Acceleration → more weight on rear → more rear grip
- Cornering → weight to outside wheels

At 1/8 scale with low mass, weight transfer effects are less pronounced than full-scale but still relevant at high speed.

### Roll Dynamics

- Roll angle tracked in state vector (state[3])
- Crash detection: `|roll| > PI/2` (tipped over)
- At high speed + sharp turn, roll can increase rapidly
- Suspension stiffness affects roll rate

## Tire Model

The neural network dynamics model implicitly learns tire behavior from data. No explicit tire model (Pacejka, etc.) is used in the MPPI controller. However, understanding tire physics helps with:
- Interpreting model behavior
- Designing cost functions
- Tuning slip angle penalties

### Tire Force Curve (Conceptual)

```
Lateral Force
    ^
    |      ___________
    |     /
    |    /
    |   /    ← linear region (small slip angle)
    |  /
    | /
    |/________________> Slip Angle
    0   peak  saturation
```

## Suspension

1/8 truggy has 4-wheel independent suspension with:
- Oil-filled shocks (adjustable damping via oil viscosity)
- Coil springs (adjustable preload)
- Long travel (~30-40 mm) — more than on-road RC cars

Suspension compliance affects:
- Camera stability (vibration → VIO noise)
- Contact patch consistency
- Roll rate during cornering

## Scaling Effects (1/8 vs Full Scale)

| Effect | Impact at 1/8 Scale |
|--------|-------------------|
| Aerodynamics | Negligible (low Re number) |
| Tire heating | Negligible (small contact patch) |
| Weight transfer | Reduced (lower CG ratio) |
| Reaction time | **Critical** — at 8 m/s, 50ms delay = 0.4m |
| Surface effects | Amplified — small bumps are proportionally larger |

## Speed Targets

| Phase | Speed | Notes |
|-------|-------|-------|
| Initial testing | 1 m/s | Verify basic track following |
| Conservative | 4 m/s | Cost function tuning |
| Target (Nano) | 8 m/s | Max safe with 50Hz control |
| Target (Orin) | 15-20 m/s | With 110Hz control |

## Dynamics Model Training

The neural network dynamics model must be trained on real driving data:

1. **Data collection:** Manual RC driving, log state + control at 100 Hz
2. **Input:** [u_x, u_y, yaw_rate, steering, throttle, dt] (6D)
3. **Output:** [du_x, du_y, dyaw_rate, droll] (4D, derivatives)
4. **Architecture:** 6→32→32→4 with tanh (same as AutoRally)
5. **Training:** PyTorch, then export to `.npz` format

Initially, AutoRally's pre-trained weights can be used for testing, but the 1/8 truggy has different dynamics (mass, tire grip, suspension) and will need its own trained model for good performance.

## AutoRally Vehicle Parameters (Reference)

From `autoRallyPlatform.urdf.xacro`:
- Wheelbase: 0.570 m
- Track width: 0.400 m
- Tire diameter: 0.190 m
- Chassis mass: 20.5 kg
- Front spring: 2627 N/m (15 lb/in)
- Rear spring: 3327 N/m (19 lb/in)
- Gear ratio: 18.68
- Max axle velocity: 282.3 rad/s

These values need to be scaled/measured for the 1/8 truggy.
