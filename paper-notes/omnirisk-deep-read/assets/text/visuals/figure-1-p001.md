# Text evidence: Figure 1

> This file contains extracted text, not a visual interpretation. The image pixels, layout, axes, colors, and panels were not verified.

- **PDF page:** 1
- **Text extraction status:** partial
- **Available sources:** caption, suggested-region-text, body-references
- **Suggested region:** [303.2, 195.503, 568.002, 384.865]

## Caption

Fig. 1. Real-world static–dynamic avoidance trial: a thrown ball crosses the quadrotor’s route through rock-shaped static obstacles. The trajectory is colored by flight time; the inset marks the closest-approach distance 𝑑min.

## Text from suggested visual region

~~~text
Fig. 1.   Real-world static–dynamic avoidance trial: a thrown ball crosses
the quadrotor’s route through rock-shaped static obstacles. The trajectory is
colored by flight time; the inset marks the closest-approach distance 𝑑min.
~~~

## Body references

- [PDF p.1] sample [1], [2], so their latency grows with dynamic obstacle count |O𝑡| and overruns control deadlines. Lightweight for- mulations [3] satisfy the timing budget by locally refining a single trajectory against proximity penalties, but in our benchmark their avoidance success drops sharply as threats become dense and fast. Faster onboard compute shortens solve times but resolves neither limitation. We resolve this trade-off by shifting continuous trajectory- risk evaluation out of the onboard flight loop and into offline learning. In simulation, each candidate primitive of an omnidirectional anchor lattice is labeled with a static clearance cost and a dynamic risk computed from ground- truth geometry and relative kinematics. At runtime, the vehicle performs no online risk integration. Asynchronous LiDAR returns and confirmed dynamic tracks are projected into a fixed-dimensional planning input tensor. A dynamic head distilled from a relative-motion risk field then scores every candidate primitive in a single forward pass, and the candidate primitive with the lowest combined static and dynamic score is selected. The corresponding trajectory is then recovered in closed form. Iterative, obstacle-dependent optimization thus reduces to a single inference whose cost is independent of |O𝑡|, and because every candidate primitive is scored, selection is not tied to one locally refined trajectory. Fig. 1 illustrates this in real-world flight, directly mapping physical elements to underlying mechanisms: the vehicle tra- verses static rock clutter via the static clearance cost; evades a

[TRUNCATED BY EXTRACTOR]

## Mandatory limitation

A text-only model must not claim direct observation of visual trends, layout, axes, colors, panels, qualitative examples, or crop completeness.
