# Spaceflight Simulator (SFS) – Orbital Launch Equations

This document defines a practical set of equations for designing and executing a launch from surface to stable orbit in Spaceflight Simulator (SFS), accounting for different difficulty settings.

---

# 1. Core Orbital Mechanics

## Vis-Viva Equation

v = sqrt( μ (2/r - 1/a) )

Where:

* v = velocity at position
* r = distance from planet center
* a = semi-major axis
* μ = GM (gravitational parameter)

---

## Circular Orbit Velocity

v_orbit = sqrt( μ / r )

This is the required horizontal velocity for a stable circular orbit.

---

# 2. Difficulty Modes (SFS Scaling)

SFS difficulty modes change **planet scale** and **vehicle performance parameters**. These directly impact required Δv, TWR, and staging efficiency.

## Normal Mode

* Scale: **1:20**
* Specific impulse: **1.0×**
* Tank dry mass: **1.0×**
* Engine mass: **1.0×**

Implications:

* Lower orbital velocity due to smaller radius
* Lower Δv requirement overall
* Standard mass fractions → moderate staging efficiency

Approximate:
Δv_total ≈ 2,500 – 3,500 m/s

---

## Hard Mode

* Scale: **1:10**
* Specific impulse: **1.0×**
* Tank dry mass: **1.0×**
* Engine mass: **1.0×**

Implications:

* Higher orbital velocity than Normal (larger body)
* Noticeably higher gravity losses
* Same propulsion efficiency as Normal

Approximate:
Δv_total ≈ 3,500 – 5,500 m/s

---

## Realistic Mode

* Scale: **1:1**
* Specific impulse: **1.5×**
* Tank dry mass: **0.25×**
* Engine mass: **0.5×**

Implications:

* Full-scale planets → **much higher orbital velocity**
* Significantly higher Δv requirement
* Improved propulsion efficiency (higher Isp)
* Much better mass fraction (lighter tanks/engines)

Approximate:
Δv_total ≈ 8,000 – 9,500 m/s

---

## Normal Mode

* Simplified drag model
* Moderate gravity losses

Δv_total = v_orbit + v_losses

Typical:
Δv ≈ 3,500 – 5,000 m/s

---

## Hard Mode (Realistic)

* Full drag + realistic gravity losses

Δv_total = v_orbit + v_gravity_loss + v_drag_loss

Typical:
Δv ≈ 6,000 – 9,000 m/s

---

# 3. Rocket Equation

Δv = v_e * ln(m0 / mf)

Where:

* v_e = exhaust velocity
* m0 = initial mass
* mf = final mass

---

# 4. Velocity Decomposition

v^2 = v_x^2 + v_y^2

Where:

* v_y = vertical velocity (altitude gain)
* v_x = horizontal velocity (orbital velocity)

---

# 5. Orbital Energy

ε = v^2/2 - μ/r

---

## Semi-Major Axis

a = -μ / (2ε)

---

# 6. Angular Momentum

h = r * v_perpendicular

---

# 7. Eccentricity

e = sqrt(1 + (2εh^2 / μ^2))

---

# 8. Apoapsis and Periapsis

r_apo = a(1 + e)

r_peri = a(1 - e)

---

# 9. Circularisation Burn

At apoapsis:

v_current = sqrt( μ (2/r - 1/a) )

v_circular = sqrt( μ / r )

Δv_circ = v_circular - v_current

---

# 10. Orbit Condition (Success Criteria)

Stable orbit requires:

r_peri > R_planet

AND

v_x ≈ v_orbit

---

# 11. Launch Profile Model

## Phase 1 – Vertical Ascent

Maximize v_y

## Phase 2 – Gravity Turn

Convert v_y → v_x

## Phase 3 – Apoapsis Creation

Set target altitude:

r_apo = R_planet + desired_altitude

## Phase 4 – Circularisation

At apoapsis:

Increase v_x until:

r_peri > R_planet

---

# 12. Optimization Objective

Minimize losses:

∫ g dt  (gravity loss)
∫ D dt  (drag loss)

Maximize:

v_x at apoapsis

---

# 13. Simplified SFS Model

You can approximate the entire launch as:

1. Build apoapsis using vertical thrust
2. Build horizontal velocity before reaching apoapsis
3. Circularise at apoapsis

---

# 14. Key Insight

Orbit is achieved when horizontal velocity is sufficient to continuously miss the planet.

NOT when altitude is high.

---

# 15. Practical Engineering Rules

* Start gravity turn early (low altitude)
* Avoid excessive vertical velocity
* Prioritize horizontal acceleration
* Perform small, precise circularisation burns

---