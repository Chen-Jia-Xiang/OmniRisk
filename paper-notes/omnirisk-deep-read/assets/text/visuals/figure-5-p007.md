# Text evidence: Figure 5

> This file contains extracted text, not a visual interpretation. The image pixels, layout, axes, colors, and panels were not verified.

- **PDF page:** 7
- **Text extraction status:** partial
- **Available sources:** caption, suggested-region-text, body-references
- **Suggested region:** [44.0, 40.089, 568.0, 388.883]

## Caption

Fig. 5. Real-world multi-threat trial in the cluttered indoor flight hall. (a) Three obstacles O1–O3 thrown in sequence; the executed quadrotor trajectory is colored by flight time, with closest-approach events 1–3 marked. (b) Surface-to-surface clearance 𝑑⊥(𝑡) to each obstacle over time (accounting for platform and obstacle radii); the vehicle maintains a safe positive margin throughout, reaching a worst-case clearance of 0.24 m.

## Text from suggested visual region

~~~text
Fig. 5.  Real-world multi-threat trial in the cluttered indoor flight hall. (a) Three obstacles O1–O3 thrown in sequence; the executed quadrotor trajectory
is colored by flight time, with closest-approach events 1–3 marked. (b) Surface-to-surface clearance 𝑑⊥(𝑡) to each obstacle over time (accounting forplatform and obstacle radii); the vehicle maintains a safe positive margin throughout, reaching a worst-case clearance of 0.24 m.

                     TABLE II                                             TABLE III
~~~

## Body references

- [PDF p.7] We deploy the trained policy with frozen weights across two novel environments—a cluttered indoor flight hall and an outdoor field—against unseen physical projectiles (a tennis ball, a foam basketball, and a plush toy) at relative encounter speeds up to 15 m/s (Table VI). Fig. 5 demonstrates continu- ous evasive maneuvers against three sequentially thrown dy-
- [PDF p.8] namic obstacles. The weakly reflective foam basketball poses the greatest challenge, inducing delayed track confirmation and tracking dropouts, and yields the lowest success rates (86.0% indoor, 82.0% outdoor). We attribute the successful trials against it to the spatial margins embedded in the dynamic risk field, which absorb perception uncertainty, and to simulation mask corruption (Sec. IV-C.2), which reduces reliance on the binary mask under tracking dropouts. In the trial of Fig. 5, the quadrotor sustained a minimum clearance of 0.24 m, indicating that the policy transfers zero-shot by relying on metric relative motion.

## Mandatory limitation

A text-only model must not claim direct observation of visual trends, layout, axes, colors, panels, qualitative examples, or crop completeness.
