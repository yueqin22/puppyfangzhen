# Stair Climbing: reaching multiple floors (exploratory)

Most homes have stairs, and a floor-bound vacuum can only clean one level. This
RFC is the home for OOMWOO's *stair-climbing* line of development — how a
build-it-yourself vacuum could move between floors and clean a whole house. It is
early and deliberately open: the point right now is to explore the design space,
prototype in simulation and CAD, and converge on an approach that fits OOMWOO's
*hackable, minimal-rebuild* ethos.

> *Status — exploratory.* A *forward-looking* capability, not a v1 promise. There
> is no committed mechanism yet — this RFC collects prior art, frames the design
> space, and invites concept + sim + CAD exploration. Develop concepts in the
> Gazebo sim ([urdf-gazebo-sim](../urdf-gazebo-sim)) and/or CAD; coordinate in the
> [discussions](https://github.com/makerspet/oomwoo/discussions) and
> [issue #52](https://github.com/makerspet/oomwoo/issues/52).

# Design direction — a drive-in "exoskeleton" (recommended)

Two broad families:

- *Onboard mobility* — give the robot itself the means to climb: legs, tank
  treads, extra or transforming wheels. **Downside:** it makes the vacuum far more
  complex and *requires rebuilding it* — every OOMWOO would carry the climbing
  hardware whether or not the owner even has stairs.
- *Drive-in exoskeleton (recommended)* — a separate **stair-climbing carrier the
  vacuum drives into**; the carrier climbs (tank/legs/etc.) with the *unmodified*
  vacuum riding inside, then the vacuum drives out to clean the next floor.
  **Why:** it is a *pure add-on* — minimal or no changes to the vacuum, a clean
  *upgrade path* for an existing build, and the climbing complexity stays isolated
  in an optional module. Commercially, Dreame's Cyber X dock with "tank legs" is
  the closest example of this idea.

A third option worth noting: a *wall-mounted rail* the robot latches onto and
slides up/down — a cheap mechanism, but it needs permanent stair-wall hardware.

The exoskeleton is the recommended direction, but this RFC is open — a compelling
onboard or rail concept is welcome; make the case.

# Prior art

Collected in [issue #52](https://github.com/makerspet/oomwoo/issues/52), grouped by
mechanism:

- *Dock / exoskeleton (drive-in)* — [Dreame Cyber X dock with tank legs](https://www.youtube.com/watch?v=EZmx_omvb3Y) (keeps the vacuum inside), plus the idea of a drive-in enclosure that ferries the robot.
- *Legged* — [Roborock Saros Rover, CES 2026](https://www.youtube.com/watch?v=b14Ic7t45lg) · [quadruped](https://www.youtube.com/watch?v=ODRyOGDc4HY).
- *Tank / tracked* — [LEGO tank, 35°](https://www.youtube.com/watch?v=mx4s8JvnPDg) · [industrial tank](https://www.youtube.com/watch?v=VljIZCixBEo) · [alt tank](https://www.youtube.com/watch?v=fv7dG0eFvwE) · [LEGO](https://www.youtube.com/watch?v=BZSkFI2wPzk).
- *Multi-wheel / hybrid* — [6-wheeler](https://www.youtube.com/watch?v=bXdt8hng2WM) · [6-wheeler](https://www.youtube.com/watch?v=3Zx7tGtwF5g) · [4-wheel front-tank + rear lift](https://www.youtube.com/watch?v=Yq45cpfJgtc) · [wheels grab the steps](https://www.youtube.com/watch?v=XzKo6KE2H5A) · [Tetris front/back wheels](https://www.youtube.com/watch?v=RwfIQpasXmA).
- *Transforming / flexible wheels* — [squishy wheels, CES 2024](https://techcrunch.com/2024/01/09/stairs-are-no-obstacle-for-this-delivery-bots-squishy-wheels/) · [triple-bent cross wheels](https://www.youtube.com/watch?v=hBFf0pZjY94) · [wheels expand](https://www.youtube.com/shorts/Wb86zJZkuvQ) · [wheels unfold](https://www.youtube.com/watch?v=bQTBavbdrss) · [folding climber](https://www.youtube.com/watch?v=8DSh4Y_wyKQ).
- *Dolly-type* — [dolly](https://www.youtube.com/watch?v=wlEDs1Eyl0Y) · [dolly](https://www.youtube.com/watch?v=fgVRNGrfnvU).
- *Dedicated stair cleaner* — [sTetro reconfigurable staircase robot](https://www.wevolver.com/specs/stetro) · [stair cleaner](https://www.youtube.com/watch?v=CznrscKo7pA).
- *Wall-mounted / climbing* — [wall-climbing suction](https://www.youtube.com/shorts/P9xPojCbLjw), plus the stair-wall rail idea above.

# Important References

- [dock-cycle RFC](../dock-cycle) — the drive-in exoskeleton is dock-adjacent (the robot drives *into* a carrier the way it docks); share the drive-in / alignment mechanics and station services.
- [urdf-gazebo-sim RFC](../urdf-gazebo-sim) — model a *staircase world* plus the carrier/robot so climbing can be tried in sim.
- [source-3d-models](../source-3d-models) · [part-specs](../part-specs) · [io-pcb](../io-pcb) — if a concept goes physical, source parts, specs and any electronics here.
- [ROS2 software interfaces](../../docs/SOFTWARE_INTERFACES.md) — the carrier↔vacuum handshake (enter → secured → climbing → arrived → exit) should be a defined contract.
- [Project discussions](https://github.com/makerspet/oomwoo/discussions?discussions_q=) · [Discord](https://discord.gg/3y2JKz5T25)

# Request for Contribution — Instructions

Early and open — concept and feasibility first, hardware later:

- *survey + concept* — pick a mechanism family (exoskeleton preferred), sketch how
  it climbs a standard interior stair (typical rise ~18 cm, run ~25 cm) while
  carrying / being an OOMWOO, and how the vacuum enters and exits. Post it in
  [Discussions](https://github.com/makerspet/oomwoo/discussions?discussions_q=) /
  [issue #52](https://github.com/makerspet/oomwoo/issues/52) so effort isn't duplicated.
- *simulate it* — build a *staircase Gazebo world* and a concept model, and show
  it climbing (physics: traction, tipping, centre of mass). A sim proof-of-concept
  is the cheapest way to compare mechanisms.
- *the docking handshake* — for the exoskeleton, define how the vacuum drives in,
  is secured, rides, and drives out — the ROS2 states *and* the physical alignment
  (coordinate with [dock-cycle](../dock-cycle)).
- *CAD / prototype (optional, later)* — model the mechanism (include source CAD;
  note tool + version), or build a bench prototype; report climb angle, payload,
  and reliability.
- *safety* — a robot hauling itself up stairs is a fall hazard; describe the
  failure modes and how a fall / tip-over is prevented or made safe. This gates any
  physical build (see [recovery-safety](../recovery-safety)).

Submit a PR to `contributions/stair-climbing/<your-github-username>/` with your
concept doc / sim world + package / CAD, install + run + test notes, and videos.
Announce it in [Project Discussions](https://github.com/makerspet/oomwoo/discussions?discussions_q=).

# Acceptance criteria

Objective where possible; at concept stage some criteria are qualitative:

- A *clearly-described mechanism* that climbs a standard interior staircase while
  moving an OOMWOO between floors, with its *upgrade-path / rebuild cost* stated.
- A *Gazebo proof-of-concept* climbing a staircase world (or a bench prototype with
  measured climb angle / payload / success rate).
- For the exoskeleton: a *defined vacuum↔carrier handshake* (enter → secured →
  climb → arrived → exit) and a physical alignment approach.
- A credible *safety story* for the fall / tip-over hazard.
- Documented and reliably reproducible by someone else.
- TBD, expect criteria to evolve.

The maintainer selects among compliant candidates using these criteria. Multiple
attempts are welcome and useful even if not selected — a non-selected concept is
still a valuable exploration and a fallback.
