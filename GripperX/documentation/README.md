# documentation

**What this is:** the GripperX documents that must version with the code, or that have to be
readable with the network down. Everything meant purely for *reading* — overviews, the user
manual, the glossary — lives in the team wiki instead, and is not duplicated here.

**Status:** 2026-08-25. One home per fact: where a document below and the wiki cover the same
ground, the document is the source and the wiki links to it.

## Documents

| File | What it is |
| --- | --- |
| `DEPLOYMENT.md` | Commissioning and bringup runbook: how the stack is brought up on the robot, the binding clean-teardown restart, and the lessons behind both. Kept here because it is needed exactly when the network is not available. |
| `ASBUILT.md` | As-**built** inventory — what is physically fitted. Pairs with `schematics/WIRING_PLAN.md`, which is as-**drawn**. Drawn is not fitted, and where the two disagree on wiring or topology the wiring plan wins: it was traced element-by-element a month after this file's inventory date. |
| `ENCODER_FEEDBACK.md` | Design and implementation record of the wheel-encoder feedback path (firmware plus hardware interface), including the measured `COUNTS_PER_OUTPUT_REV`. |
| `TWIN_OCTOPUS_RUNBOOK.md` | How to run the Octopus link against the digital twin. Versions with the world files it names. |

## Octopus interface

The Octopus is the external litter-detection system that supplies GPS goals to the robot. The ROS
side of the link is implemented in
[`Software/ros2/src/gripperx_external/`](../Software/ros2/src/gripperx_external); its API is
documented on the Software Overview wiki page.

| File | What it is |
| --- | --- |
| `OCTOPUS_INTERFACE_PROPOSAL.md` | The message and topic contract. Its *"Agreed"* section records what the exchange of 2026-08-20/21 settled and supersedes the proposal sections it names; the rest is still proposal. |
| `OCTOPUS_DECISION_BRIEF.md` | The open decisions on that contract and their trade-offs. |
| `OCTOPUS_ROSBRIDGE_SETUP.md` | How to bring the rosbridge link up on both sides — the transport the link uses. |

The correspondence with the Octopus team is **not** part of this repository and must not be cited as
the contract. Its outcomes are folded into the *"Agreed"* section above; the letters themselves are
archived internally.

## `schematics/`

| File | What it is |
| --- | --- |
| `WIRING_PLAN.md` | The as-drawn pin, cable and terminal plan — the source of truth for what should be wired, and for the drawings below. |
| `PIN_TEST_CHECKLIST.md` | Bench procedure for verifying the wiring against that plan. |
| `overview.drawio` | Block-level power and signal overview. |
| `detail.drawio` | Detailed wiring diagram. |

Both `.drawio` files are edited with [diagrams.net / draw.io](https://www.diagrams.net/).
