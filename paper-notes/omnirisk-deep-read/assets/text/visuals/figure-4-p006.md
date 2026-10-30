# Text evidence: Figure 4

> This file contains extracted text, not a visual interpretation. The image pixels, layout, axes, colors, and panels were not verified.

- **PDF page:** 6
- **Text extraction status:** partial
- **Available sources:** caption, suggested-region-text, body-references
- **Suggested region:** [44.0, 40.083, 308.802, 168.972]

## Caption

Fig. 4. Quadrotor platform: (1) Livox Mid-360 LiDAR, (2) NVIDIA Jetson Orin NX, (3) NxtPX4v2 flight controller.

## Text from suggested visual region

~~~text
Fig. 4.  Quadrotor platform: (1) Livox Mid-360 LiDAR, (2) NVIDIA Jetson
Orin NX, (3) NxtPX4v2 flight controller.
~~~

## Body references

- [PDF p.6] Each trial requires the quadrotor to navigate from a fixed start to a goal through static clutter and dynamic projectiles. A trial is counted as successful only if the vehicle reaches the goal collision-free within the allotted time; physical contact and kinodynamic infeasibility are recorded as failures. Given our 20 Hz replanning loop, we define a deadline miss as any cycle in which planning latency exceeds the timing budget Δ𝑇budget = 50 ms, forcing the low-level controller to continue tracking the previously committed trajectory; misses are counted per cycle and do not by themselves fail a trial. Planning latency is measured from planning-input arrival to trajectory commitment, covering tensor rasterization and network inference; upstream sensing, detection, and track confirmation are shared by all methods and excluded. Closed- loop simulations run in real time, and all methods share 50 paired evaluation seeds per condition, disjoint from the training seeds; key success rates (SR) are reported with Wilson 95% confidence intervals (CIs). The entire pipeline runs onboard an NVIDIA Jetson Orin NX under realistic multi-process resource contention on the custom quadrotor shown in Fig. 4. We supply identical synchronized inputs to the state-of-the-art baselines FAPP [3] and SIMP [2].

## Mandatory limitation

A text-only model must not claim direct observation of visual trends, layout, axes, colors, panels, qualitative examples, or crop completeness.
