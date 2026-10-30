# Visual inventory

> Automatic crops are candidates. Visually inspect every asset before publication.

| ID | Label | PDF page | Caption | Candidate crop | Review |
|---|---|---:|---|---|---|
| figure-1-p001 | Figure 1 | 1 | Real-world static–dynamic avoidance trial: a thrown ball crosses the quadrotor’s route through rock-shaped static obstacles. The trajectory is colored by flight time; the inset marks the closest-approach distance 𝑑min. | `crops/figure-1-p001.png` | VISUAL REVIEW REQUIRED |
| figure-2-p003 | Figure 2 | 3 | System architecture. Offline, the candidate primitives Π are supervised by the static clearance cost and the asymmetric dynamic risk field (C2, dashed arrows). Online, streaming perception is rasterized into the unified planning input tensor 𝑰𝑡(C1); a single forward pass over the omnidirectional anchor lattice A scores all candidate primitives for 20 Hz trajectory commitment without online optimization (C3). | `crops/figure-2-p003.png` | VISUAL REVIEW REQUIRED |
| figure-3-p005 | Figure 3 | 5 | Asymmetric dynamic risk field in the horizontal plane. The quadro- tor at the origin (velocity 𝒗0, blue) targets the goal (green), encountering an obstacle (velocity 𝒗𝑜, red) at 𝑑= 2.2 m. Background: pointwise dynamic risk 𝑟𝑜under relative obstacle motion. Candidate primitives are colored by dynamic risk 𝐽𝑑,𝑖(9). | `crops/figure-3-p005.png` | VISUAL REVIEW REQUIRED |
| figure-4-p006 | Figure 4 | 6 | Quadrotor platform: (1) Livox Mid-360 LiDAR, (2) NVIDIA Jetson Orin NX, (3) NxtPX4v2 flight controller. | `crops/figure-4-p006.png` | VISUAL REVIEW REQUIRED |
| figure-5-p007 | Figure 5 | 7 | Real-world multi-threat trial in the cluttered indoor flight hall. (a) Three obstacles O1–O3 thrown in sequence; the executed quadrotor trajectory is colored by flight time, with closest-approach events 1–3 marked. (b) Surface-to-surface clearance 𝑑⊥(𝑡) to each obstacle over time (accounting for platform and obstacle radii); the vehicle maintains a safe positive margin throughout, reaching a worst-case clearance of 0.24 m. | `crops/figure-5-p007.png` | VISUAL REVIEW REQUIRED |
