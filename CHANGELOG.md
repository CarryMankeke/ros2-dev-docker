# CHANGELOG

Todos los cambios notables en este proyecto serán documentados en este archivo.

## [Unreleased]

### Added
- ✨ GitHub Actions CI/CD pipeline (`.github/workflows/ci.yaml`)
  - Automated build validation with `colcon build --symlink-install`
  - Linting checks (flake8, YAML validation)
  - Smoke test of launch file syntax
  - ROS 2 Jazzy compatible
- 📝 Comprehensive audit report (`AUDIT_REPORT.md`)
  - 5 critical issues identified
  - 5 major issues identified
  - Prioritized roadmap (immediate, short-term, medium-term, long-term)

### Fixed
- 🔧 `joint_state_aggregator.py` now has executable permissions (`chmod +x`)
- 🐳 Docker container runs as non-root user `ros` (security hardening)
  - User created with sudo access for flexibility
  - Proper ownership of supervisor directories
- ⚙️ Dynamic parameter passing for `mecanum_drive_controller` spawner
  - `wheel_separation_x`, `wheel_separation_y`, `wheel_radius` now scale with `base_scale` parameter
  - Fixes inconsistency when using `scale` argument != 1.0

### Changed
- 📦 Dockerfile
  - Added `USER ros` directive (non-root execution)
  - Added user creation and permissions setup
  - Improved security posture

### Documentation
- 📚 AUDIT_REPORT.md created with full analysis
- 📋 CHANGELOG.md created (this file)
- ✅ All executable scripts verified to have shebang `#!/usr/bin/env python3`

---

## [0.1.0] - 2025-12-27

### Initial Release
- Mobile manipulator simulation stack (mm_base + mm_arm) on ROS 2 Jazzy + Gazebo Harmonic
- Docker container with noVNC support
- Launch system with multiple operational modes (sim, display, nav2, moveit, teleop)
- Joystick teleoperatione support (mm_base, mm_arm 6DOF, gripper)
- Basic documentation (README.md, info.txt, docs/)

---

## Notes

- **Format:** Follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)
- **Versioning:** Follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html)
- **Next Release Target:** v0.2.0 (planned for mid-January 2026)
  - SRDF parametrization (C5)
  - Exact version pinning (M2)
  - Dynamic joint name extraction (M3)
  - Launch tests integration
