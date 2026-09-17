# Dust Bin (mechanical module)

> *On hold.* The project moved *away from the old teardown reference vacuum*, so the
> dust bin must be designed around the *sourced parts + a forthcoming 3D reference design*
> (not yet sketched). This module resumes once the key parts are sourced and the design is
> sketched. *Useful now:* [source-3d-models](../source-3d-models) and [part-specs](../part-specs).

A removable dust bin that collects dust, holds an air filter, and receives air from the
vacuum fan. The bin inlet mates with a gasket; the lid latch is spring-loaded.

# Request for Contribution — Instructions (resumes when off hold)

When this reopens, design a 3D-printable removable dust bin that:
- mates (sealed) with the *vacuum-fan inlet* and the chassis
- holds a *common, replaceable filter* — standardise on an abundant filter size so builders
  can rebuy anywhere (see [design research](../../README.md#design-research))
- has a reliable *spring latch*; easy to remove, insert, and empty
- is 3D-printable (assume PETG); split to fit ~20 × 25 cm, 20 cm height
- ignore mop functionality for now
- submit STEP + native CAD + 3MF/STL + a sub-component BoM + docs/photos to
  `contributions/dust-bin/<your-github-username>/`

## Acceptance criteria (when it resumes)

- *Sealed* airflow (no leaks), easy to empty, reliable latch
- Fits the (forthcoming) chassis + fan interfaces without forcing changes to other modules
- Uses a common, sourceable filter size
- Documented; STEP + native CAD source provided
- TBD, expect criteria to evolve

The maintainer selects among compliant candidates using these criteria. Multiple attempts
are welcome and useful even if not selected — modules are swappable, and a non-selected
design is still a valid learning exercise and a fallback.
