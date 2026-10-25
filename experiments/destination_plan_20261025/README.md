# B/C/D destination references and fill order

Implements the operator's destination layout as a small pure-data helper in
`challenge2/destination_plan.py`. It replaces no legacy controller, perception
code, original taught poses or simulator-truth `SortPlan`.

- **B** uses #11 XY and receives requested accepted colors.
- **C** uses #14 XY; **D** uses #15 XY and receives remaining accepted colors.
- Every first-layer nominal EE Z is trusted #3 Z minus nine 9.6 mm intervals:
  **−3.973287 mm in robot base**. Old #8–15 Z/rotations are not used to set targets.
- Each layer has four ridges of three end-to-end, stud-aligned 2×4 bricks.
  The operator confirmed two empty stud rows between ridges and a common center
  when successive layers alternate by 90°.

## Indexing and geometry

`b_n_m_k`, `c_n_m_k`, `d_n_m_k`: `n=0…3` is the ridge, `m=0…2` is the brick along
it, `k=0…24` is the layer from zero. The confirmed-placement count determines
`count = 12*k + 3*n + m`. Fill each ridge first, then the next ridge, then the
next layer: `b_0_0_0`, `b_0_1_0`, `b_0_2_0`, `b_1_0_0`, …, `b_3_2_0`, `b_0_0_1`.

Layer-zero long axes point away from the image bottom, consistent with the
source lattice axes and the #11→#12 XY direction. The positions use the trusted
source position-grid basis. Parity swaps the positive long/across index axes
around the common whole-layer center, so the first anchor of the odd layer
shifts by half a center pitch along each opposing grid direction. It does not
repeat the original corner at every height or reverse the canonical indices.

Use **32 mm center pitch**, from the native 8 mm stud grid. The older source
export's 31.8 mm inferred pitch remains untouched and is not used to generate
these placement coordinates. A nominal 31.8 mm body would leave approximately
0.2 mm end clearance on a 32 mm grid; the operator's touching description is
represented as adjacent stud-aligned bricks, not compressed stud spacing.
Nominal footprint envelopes are 32×16 mm; actual plate/tape fit is unverified.

New proper downward EE rotations align their long-axis line with that grid,
choosing the equivalent 180° sign closest to the source parity's accepted
orientation. This introduces approximately 1.21° relative to the imported
source tool orientations, separately from the requested 90° alternation.
The old #8–15 rotation review reports both parity alternatives and nearest XY
slots; it does not blindly rotate all eight records or infer their old layers
from untrusted Z. #11/#14/#15 are operator-designated first-layer XY anchors.

All positions are **nominal configured-EE targets in `robot_base_O`**. The
physical held-brick transform, actual EE centering, seating offset, width/mount
mapping, clearance and hardware registration are unverified. These targets
are neither model-grip coordinates nor calibrated brick poses. Original
`O_T_EE` data and historical validation flags are preserved separately.

## Routing and confirmation

Accepted palette order is `dark_red`, `brown`, `tan`. Requested colors route to
B; remaining colors occupy C then D in that stable palette order. Thus one
requested color uses all three destinations; two use B/C; three use only B.
`None`, `UNKNOWN` and unrecognized labels abstain. Only existing explicit reject
labels `reject_4211201` / `reject_6135191` route to REJECT; this helper adds no
quality classifier or automatic bright-red/ochre mapping.

For mixing, the preferred next B color cycles through the requested list by
confirmed B count. The caller must select a suitable available brick; routing
alone cannot guarantee balance when that color is unavailable. No general
scheduler or source-ground-truth access is introduced.

`slot()` proposes without changing counts. `confirm(destination, expected_count)`
advances only when explicitly called, rejecting stale counts and overflow.
It does not itself verify physical placement, resting versus native attachment,
or an online completion signal. Those remain downstream requirements. No
transfer/stacking success is inferred from these data structures.

The finite **25-layer / 300-slot capacity per destination** covers the worst
case of all 300 input bricks going to one destination. It is a data export
limit, not authorization or evidence for a reachable or stable 25-layer stack.
All demonstrations still need a separate small motion/contact bound.

## Reproduce

```bash
/home/faps/miniforge3/envs/unitree_sim_env/bin/python -m unittest discover -s tests -p test_destination_plan.py -v
/home/faps/miniforge3/envs/unitree_sim_env/bin/python experiments/destination_plan_20261025/evaluate.py --self-test
python3 experiments/destination_plan_20261025/submit.py
```

Synthetic tests use invented coordinates only. The submitter snapshots the
helper, dependencies, scripts, configuration and hash-pinned reference export;
then submits export and independent audit as one **default Pueue** job, with a
25 s process timeout plus 2 s kill grace. No simulation, rendering, training,
ROS integration or hardware control is started. The image output is a plan
schematic, not an Isaac render or a calibrated photograph overlay.
