# System card

**System:** Vision-guided robotics, version 2.0.0. **Purpose:** reproducible engineering education and portfolio evaluation.

**Implemented:** ArUco pose estimation, planar-pose ambiguity reporting, calibrated frame composition, and ROS 2 visualization.

**Data:** bundled previews and sample metrics come from programmatically generated inputs. External datasets must be supplied by the user with appropriate permission. No external benchmark has been evaluated in this release.

**Generalization:** results on synthetic data do not establish performance on new cameras, factories, materials, equipment, or populations. Inspect domain shift and false positives/negatives before any operational use.

**Status:** core regression and demo evidence is recorded in VALIDATION.md. Optional pretrained SAM, ROS 2 runtime and physical robot integrations are not certified or assumed tested. CI configuration is supplied, but hosted CI results exist only after a repository actually runs it.

**Human contribution:** code and documentation were developed with AI assistance. Review, reproduce, and document your own experiments before making technical claims in a CV or interview.
