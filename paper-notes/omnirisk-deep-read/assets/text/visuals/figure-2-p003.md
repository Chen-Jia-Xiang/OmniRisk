# Text evidence: Figure 2

> This file contains extracted text, not a visual interpretation. The image pixels, layout, axes, colors, and panels were not verified.

- **PDF page:** 3
- **Text extraction status:** partial
- **Available sources:** caption, suggested-region-text, body-references
- **Suggested region:** [44.0, 40.084, 568.019, 300.07]

## Caption

Fig. 2. System architecture. Offline, the candidate primitives Π are supervised by the static clearance cost and the asymmetric dynamic risk field (C2, dashed arrows). Online, streaming perception is rasterized into the unified planning input tensor 𝑰𝑡(C1); a single forward pass over the omnidirectional anchor lattice A scores all candidate primitives for 20 Hz trajectory commitment without online optimization (C3).

## Text from suggested visual region

~~~text
Simulation source     Shared trunk · identical in both domains     C2 · Asymmetric dynamic risk field                   training-time only
    inverse-CPA sampling
                     C1 · Unified planning input tensor              amplified                  decays
                                         five channels · one H×W grid                      ahead                     behind


                                                                                          velocity-aligned, causally asymmetric           directed path integral
                                                                                             closing amplified · receding → 0                 one scalar per primitive

   Real-world source                                       C3 · Online stage                          no online optimization
   MID-360 · FAST-LIO2
                          Dual-branch CNN · one forward pass

                                                                         Score fusion
                                              Static                                         head                       S = Sstatic + λ Sdyn                                        36 endpoints

                                   Dynamic head                         36 scores fused        endpoint committed            quintic · 50 Hz
                                                          risk scores only
                             backbone                                        O(N) selection       20 ms inference      15 m/s zero-shot

                                                            data flow            analytic supervision, training only

Fig. 2.  System architecture. Offline, the candidate primitives Π are supervised by the static clearance cost and the asymmetric dynamic risk field (C2,
dashed arrows). Online, streaming perception is rasterized into the unified planning input tensor 𝑰𝑡(C1); a single forward pass over the omnidirectional
anchor lattice A scores all candidate primitives for 20 Hz trajectory commitment without online optimization (C3).
extent 𝒔    (𝑑   𝑑   𝑑   ) ∈R3 and persistent tracking    primitives violating the kinodynamic limits ∥𝒗(𝑡)∥ ≤𝑣
~~~

## Body references

- [PDF p.3] 1) Offline Stage: Fig. 2 summarizes the resulting two- stage architecture. The offline phase constructs the super- vision needed to bypass real-time numerical optimization. Within the privileged simulation domain Dsim, the train- ing pipeline samples randomized dynamic obstacle encoun- ters and accumulates collision risk along candidate primi- tives. Specifically, inverse-CPA sampling first draws a target closest-approach distance and time (𝑑CPA, 𝑡CPA) and back- computes the obstacle’s initial state, concentrating training samples in near-collision regimes. A compact neural network is trained via supervised regression to predict this continuous trajectory risk directly from the synthesized planning input tensor 𝑰𝑡, embedding non-convex collision reasoning into the network weights prior to deployment and eliminating online numerical integration. 2) Online Stage: The onboard framework comprises three asynchronously decoupled modules: an omnidirectional per- ception front-end, the trained dual-branch circular convolu-

## Mandatory limitation

A text-only model must not claim direct observation of visual trends, layout, axes, colors, panels, qualitative examples, or crop completeness.
