# Revisable Challenge 2 demo protocol

This supplements the six-stage simulation goal. It does not promote any
unmeasured candidate parameter to a validated method. The measured state is in
[demo_progress.md](demo_progress.md): ordinary top-view candidates/mock joins
pass, sparse remnants fail, and the full 300-brick native idle gate fails before
manipulation. Extra compute is optional and does not fix that model failure.

## Experiment discipline

Before executing a stage, freeze its cases, configuration, numeric motion/contact
limits, completion/failure rules, metrics, resource budget and reserved subset.
Use a baseline and at most **three reviewed improvement rounds**; each round
changes one observed-failure mechanism. Reassess at the limit, or earlier if the
method needs broader work. Renaming a camera/benchmark does not reset the count.
Log bounded diagnostic runs separately, with a predeclared run/step/time budget;
they are not unlimited free attempts. Preserve regressions, retries, misses and
uncertainty. Reserved observations cannot drive changes within the same round.

For accounting, Stage 1 already used two substantive changes (workspace filter,
closer views). Stage 2 used one coupled numerical-epsilon comparison, which did
not fix its failed idle gate. Native logging/offline audits are diagnostics, not
additional claimed improvements. No unbounded sweep is planned; each new experiment still needs its frozen
budget and independent review.

## Classification observation timing

**Timing is not yet selected.** Compare before-grasp/source inspection against
stationary held-brick inspection before choosing. The C920 housing and fascia
are visual proxies; they do not establish optics, focus or visibility of a held
brick. Existing source detection accuracy does not establish classification crop
adequacy. Challenge 1 must identify which region/face its classifier needs.

A compact proposed reference set is three identified brick examples observed
at both timings by D435i and C920 (12 RGB observations), plus one empty-hand/source
negative per camera. Reserve one whole brick and its four observations from
implementation choices. Prefer existing captures; if physical images must be
provided, a person can present examples near stationary fingers. This does not
authorize robot or camera hardware control. Rendered fixture images can test
geometry under declared provisional optics, but cannot verify physical focus.
Manually initializing a held fixture is not a measured grasp or extraction.

Measure required-face visibility, finger/stand occlusion, brick/stud pixel scale,
blur, glare and selected/held-brick association. Record ambiguous associations
and missing crops. Choose one timing only if the development comparison and
reserved check support usable, unambiguous inputs; otherwise abstain and leave
timing unresolved. Freeze any numeric crop-quality screen before scoring;
unknown classifier requirements cannot be filled with arbitrary thresholds.

Use one bounded classification request at that timing, with an exact observation
and pick/presentation association. A new held observation is not silently joined
to an old source frame: preserve an explicit transaction link and invalidate it
on slip, regrasp or stale observations. Missing/duplicate/unknown results abstain.
Allow at most two captures and one request per attempted brick. Until a real
Challenge 1 interface is available, provider and results remain explicitly mocked;
never use simulator colors or quality labels as classifier inputs.

## Extraction and contact prerequisites

First pass the full-source native idle gate. Register the supplied tip model's
collision representation and material separately from its current visual fit.
Verify contact sensing and units before relying on force limits; width/closure
alone is not proof of a grasp. Native component truth remains evaluator-only.

In a small frozen contact benchmark, estimate and check `T_TCP_brick`, the held
brick's pose relative to the gripper/TCP. Log grasp depth, finger opening,
relative slip and swept finger clearance using the actual pad geometry. Do not
assume stock TCP offsets still describe the changed pads. A candidate can be
visually exposed yet physically inaccessible.

The earlier **25° tilt and 5 mm withdrawal are unverified trials**. Declare the
brick's long/short axes and outward top normal in the table frame. Distinguish
TCP-centered rotation from a bottom/contact-edge peel: for an edge pivot, compute
the TCP path using the held transform and estimated edge, not an in-place TCP
rotation relabeled as peeling. A fixed edge hypothesis is not a measured contact
pivot. Withdrawal candidate is 5 mm along the *tilted brick's outward top normal*,
not automatically gripper Z; record frame, sign and actual observed displacement.

Start with one exposed-brick/contact configuration and a reserved offset or
support configuration. Change one primitive parameter per improvement round.
Before any run, freeze SI force/torque thresholds, contact travel, tilt range,
withdrawal distance, speed, dwell and timeout ceilings. Unavailable force data
is unknown, not zero force. Permit at most one renewed whole-brick attempt after a safely recovered
failure, within the global eight-attempt budget; abort on lost association, slip, exceeded
bounds, ambiguous completion or observed neighboring-stack disturbance. Simulator
truth independently checks intended detachment and collateral native breaks.
The current idle failure prohibits treating a spontaneous break as pick success.

## Placement and stacking

Use the configured/calibrated receiving-baseplate frame and explicit stud-grid
coordinates, excluding taped rows and zone boundaries. Convert a desired brick
pose to a TCP target through `T_TCP_brick`; include receiver height, orientation,
layer height and uncertainty. An empty-plate receiving surface differs from a
previously placed brick. Determine the receiving top/grid from observations and
placement history with verification, not privileged target poses.

Visual refinement precedes gentle contact with frozen travel/force/torque limits.
Check swept finger clearance through seating and release. Do not presume that
a side grasp permits full seating; a release/press alternative needs its own
contact test if clearance fails. Bound lateral refinement and attempts; do not
build a general search/scheduling system.

Keep **resting on studs**, **observed seated**, **native attached** and **uncertain**
outcomes separate. Online completion needs defined image/proprio/contact evidence
for seating, release and stability; attachment needs an explicitly tested observable
(e.g. a bounded retention check). Force increase or a plausible image alone is
insufficient. Native connection creation, intended grid/yaw and retained attachment
are evaluator-only ground truth. Until online feedback is validated against that
truth, report attachment unknown. Never set a connection to declare success.

## First bounded recipe

Use at most six processed input bricks and eight total manipulation attempts,
including retries. One attempt is a complete brick-handling try from approach
to verified destination or abort; individual waypoints do not consume separate
attempts. A failed try can be renewed only after observed safe recovery and
reassociation, never by post-reset teleportation. Target four accepted bricks in a small fit-checked 4×2-stud
column, so the demo includes placement on the plate and on an existing brick.
Include a reject case and, when applicable, an unrequested-good-color buffer case.
These counts are a proposed first fixture, not a guarantee of source availability.

Single-color requests repeat that label. Mixed requests use a fixed cyclic order
of the requested labels, chosen once in configuration (for example red/tan/red/tan).
No open-ended optimizer or general scheduling system is needed. If the next
required color has no usable accessible candidate/receiving slot, abstain or end
the bounded demo rather than silently changing the arrangement. Unrequested good
labels go to their fixed small zone; faulty bricks go to reject. Unknown labels
are not treated as faulty. When all good labels are requested, buffers stay empty.
Freeze a representative input/label sequence and a small reserved mismatch case;
explicit mock sequences remain interface tests, not actual classification success.
