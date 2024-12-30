
# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2024-12-19

First official public release.

### Added

- Docker image robot-l4t for Jetson AGX
- Automatically load and play Isaac Sim scene upon launch
- Random walk planner
- DROAN trajectory-library based local planner
- Initial GCS rviz capable of visualizing multiple robots

### Fixed

- A bunch of stuff honestly

### Changed

- Upgrade Isaac Sim from 4.1.0 to 4.2.0
- Unified docker image naming to use AirStack's version.
- Condensed GCS TAK docker images to single docker image

### Removed

- Duplicate TAK images
