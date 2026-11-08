# Vision-guided robotics

**Hazem Nefzi · Robotics / AI / Computer Vision · v2.0**

ArUco pose estimation, planar-pose ambiguity reporting, calibrated frame composition, and ROS 2 visualization.

![Working demonstration](assets/demo.png)

## Start in three commands

Use an isolated Python environment for each repository. From this directory:

```bash
python -m pip install -e ".[dev]"
python -m robotics_vision.demo --output outputs
python -m pytest -q
```

For Windows setup, see [the installation guide](docs/INSTALL.md). The package can be imported from outside the repository after installation.

## What this repository demonstrates

ArUco pose estimation, planar-pose ambiguity reporting, calibrated frame composition, and ROS 2 visualization. This is an engineering portfolio implementation with explicit failure handling and reproducible experiments. Sample results are synthetic; no external benchmark ranking is claimed.

| Workflow | Command / action |
|---|---|
| Basic demonstration | `python -m robotics_vision.demo --output outputs` |
| Extended experiment | `python -m pytest tests/test_robustness.py -q` |
| Own data / integration | `python ros2_publish_pose.py --pose outputs/poses.json` |
| Static checks | `python -m ruff check .` |
| Regression tests | `python -m pytest -q` |

## Evidence and engineering notes

- [Validation report](docs/VALIDATION.md): executed tests, environment, and limitations.
- [Design decisions](docs/DESIGN.md): algorithms, assumptions, and failure modes.
- [Detailed usage](docs/USAGE.md): extended workflows and input contracts.
- [Model / system card](docs/SYSTEM_CARD.md): intended use and evaluation limits.
- [Contributing](CONTRIBUTING.md): development and review expectations.

## Scope and attribution

This repository was developed with AI assistance and should be reviewed and understood before use. The original synthetic examples are independent of confidential internship or research work. They are not a release of a deployed industrial system. Optional external models and datasets retain their own licenses.

MIT license applies to this repository's original code. See `LICENSE`.
