# Preliminary design — m₀ = 29 724 kg, range 12 418 km

Untracked working folder. Nothing in the repo was modified; `main.py` and
`traj/rocket.json` are still at HEAD. Apply the changes below to reproduce.

## Result

| | value | requirement |
|---|---|---|
| launch mass m₀ | **29 724 kg** | 29 700 kg (Minuteman I, `archive.typ:108`) |
| full range L | **12 418 km** | 12 000 km (`archive.typ:183`, +20 % on 10 000) |
| burnout | V 7 362 m/s · H 181.5 km · θ 20.2° | table 2.1 at 12 000: 7 150 m/s, 250 km, 23° |
| achieved k_V | **1.1837** | design K_V 1.198 — measured beats design |

HEAD for comparison: 39 523 kg / 12 053 km. This is −9 799 kg (−24.8 %) with more range.

## Changes to `main.py`

| line | from | to |
|---|---|---|
| 36 `L_FULL` | 12053 | 12000 |
| 59 `K_V` | 1.25 | 1.198 |
| 108 `ALPHA_C` | 0.005 | 0.004 |
| 111 `A_OMEGA_3` | 0.025 | 0.015 |
| 112 `N_TAIL` | 0.012 | 0.008 |

Unchanged: p_k 50/35/35 bar, p_a 0.70/0.37/0.14 bar, ETA 1.2, EPSILON 0.99,
ALPHA_BR 0.07, D_K_BAR 0.30, BETA_C_DEG 20, K_S 2.03, N_NOZZLES 4,
lambda_z at chart 4-27 values, fuels PB/PU/PU, materials.csv, payload 620 kg.

Plus `traj/rocket.json`: pitch program 8 arcs -> **14 arcs (6/4/4)** —
see `rocket-29724.json`, which is a drop-in replacement.

## Why K_V = 1.198 works

`main.py:45-52` states that no self-consistent k_V exists inside the Appazov
1.15-1.25 band, and sizes at 1.25 for that reason. That conclusion is an
artifact of an under-parameterised pitch program. Arc count was worth range
at every step tested: 8 -> 10 -> 12 -> 14 arcs. With 14 arcs the achieved
loss factor is 1.1837, comfortably inside the band and near the section 5.2
worked example's 1.165.

## Open items

- lambda_2 = 0.240 and lambda_3 = 0.165 sit below their bands
  (0.25-0.30, 0.18-0.20). HEAD already had lambda_3 out at 0.170.
- `A_OMEGA_3` 0.015 and `N_TAIL` 0.008 carry ~2 690 kg between them and are
  flat "prinimaem" values in the book with no stated interval. `ALPHA_C` 0.004
  is fine — bottom of the stated 0.004-0.008 range.
- Condition (3.43) fails on stage 3: d_m3 = 0.829 < d_a3(1+sqrt2) = 1.07.
  A single nozzle changes the condition to d_m >= d_a (0.829 >= 0.443).
- Three of four section 4.4 limits sit exactly on their bound (CON_MARGIN = 1e-3).
  No robustness margin on alpha or pitch rate until the CFD is re-run.

## Aerodynamics

The mass reduction brought the design back onto the CFD geometry. `rocket.scad`
(deleted in `8cf4bf3`) meshed d_m = [1.58, 1.17, 1.17]; this design is
[1.576, 1.164, 0.829] — stages 1 and 2 within 0.5 %. `config.go`'s
RrefAll = 0.795 is now only +1.8 % off the true half-diameter, against
-17.1 % at HEAD.

Stage 3 remains 29 % narrower than what was meshed, so `stage3up` and `head`
coefficients are the ones to redo.

## Files

- `rocket-29724.json` — drop-in `traj/rocket.json`
- `traj-29724.csv.gz` — full trajectory, h = 0.1 s
- `charts/` — pitch program, trajectory, section 4.4 limits
- `scripts/` — chart generators; `remote_run.py` is the parallel CMA-ES driver

## Reproduce

```bash
cd traj && go build -o out/traj-sim ./main
./out/traj-sim -config=../preliminary-29700/rocket-29724.json \
               -aero=../openfoam/results/averages.csv -out=out/traj.csv
```

Optimiser settings that mattered: `--target 14000` (drives maximisation rather
than hitting 12 000 exactly), `--maxiter` >= 1500 (500 was severely
under-converged and produced a frontier that was wrong by ~1 500 km), and
best-of-N over 6+ seeds — the landscape is strongly multimodal.
