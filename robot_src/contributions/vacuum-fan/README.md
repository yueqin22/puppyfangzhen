# Blower Fan Assembly (mechanical module)

> *On hold.* Fan options are *already sourced* — see the suction-fan rows in
> [BOM.md](../../BOM.md) (multiple Pa/price options: Dreame, Nidec, Roborock-family blowers).
> The project has also moved *away from the old teardown reference vacuum* (its parts are
> hard to source vs. big names like Roborock/Dreame/Xiaomi). This module resumes once enough
> components are sourced and a *3D reference design with the key parts* is sketched, so the
> housing can be designed to fit real parts.
>
> *Useful upstream work now:* model the sourced fans in
> [source-3d-models](../source-3d-models), and gather fan specs / how to drive them in
> [part-specs](../part-specs). Those unblock this design.

The vacuum fan assembly provides air suction to the main brush and streams air into the dust bin.
It consists of:
- a high-speed *BLDC blower motor + impeller* (already sourced — see BOM)
- a blower housing (volute) — to be designed
- a blower exhaust gasket
- blower housing rubber mounts

> *Design research note:* real-world cleaning does *not* track raw suction (Pa) —
> mid-range sealed motors match flagships. Prioritise a *well-sealed* airflow path
> (no leaks at the bin / fan / brush seams) and a good brush over chasing maximum Pa.
> See [design research](../../README.md#design-research).

# Request for Contribution — Instructions (resumes when off hold)

When this reopens, the task is to design a *3D-printable volute housing + gasket around a
sourced BLDC blower* (chosen from the [BOM](../../BOM.md)) that mates cleanly with the dust
bin and the chassis. Design goals:
- 3D-printable (assume PETG); ideally no supports; reliably reproducible
- sliced part fits ~20 × 25 cm, 20 cm height (split if needed)
- *well-sealed* airflow path (no leaks); mates with the dust bin + chassis
- low vibration; secure motor mounting
- gasket: find an off-the-shelf one, else print TPU
- submit STEP + native CAD + 3MF/STL + a sub-component BoM + docs/photos/video to
  `contributions/vacuum-fan/<your-github-username>/`

## Acceptance criteria (when it resumes)

- Fits the (forthcoming) chassis + dust-bin interfaces without forcing changes to other modules
- Well-sealed, reasonably quiet, low-vibration, strong suction with the chosen sourced blower
- 3D-printable and reliably reproducible by someone else
- Documented; STEP + native CAD source provided
- TBD, expect criteria to evolve

The maintainer selects among compliant candidates using these criteria. Multiple attempts
are welcome and useful even if not selected — modules are swappable, and a non-selected
design is still a valid learning exercise and a fallback.
