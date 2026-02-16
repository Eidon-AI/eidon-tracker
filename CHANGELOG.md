# Changelog

All notable changes to this project will be documented in this file.

This project follows [Semantic Versioning](https://semver.org/).

## [2.1.0] – 2026-10-13
### Added
- Battery reporting for the left side devices

### Changed

### Fixed

### Removed
- 

## [2.0.1] – 2026-10-13
### Added

### Changed

### Fixed
- Fixed low child transmition rate errors

### Removed
- 


## [2.0.0] – 2026-01-10
### Notes
Not backwards compatible. Requires App version 2.0.0+

### Added

### Changed
- Major reworking of hub <> child architectue
- Right sides are now hubs, all left sides connect to right side

### Fixed
- This fixes Android BLE limitations

### Removed
- 

## [1.0.1] – 2026-01-10
### Added

### Changed

### Fixed
- Calibration Error
- Reset all services to fully reset the state

### Removed
- Battery debug

## [1.0.0] – 2026-01-08
### Added
- Raw data support (required by app)
- Central versioning
- Battery monitoring for battery-aware devices (Batch 2+)

### Changed
- App compilation size to accomodate a larger firmware

### Fixed
- 

### Removed
- Battery debug

---

## [0.1.0] – 2025-10-01
### Added
- Initial firmware scaffolding
- BLE services and characteristics
- Basic device configuration and streaming
- Batch one device participation
