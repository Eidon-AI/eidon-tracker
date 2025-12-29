# Eidon Tracker Versioning

We use Semantic Versioning: MAJOR.MINOR.PATCH (e.g., 1.4.2).

- PATCH: bug fixes only, no behavior changes that would surprise users
- MINOR: backwards-compatible features and improvements
- MAJOR: breaking changes to defined compatibility contracts

## What is “compatible” in Eidon Tracker?

A change is considered BREAKING (MAJOR bump) if it breaks any of:
- Firmware BLE services/characteristics UUIDs, formats, or required fields
- On-disk file formats (sessions, recordings, exports) without migration
- Tracker ↔ hub protocol expectations
- Hub ↔ App expectations

A change is NOT breaking if:
- It adds new optional fields (with sensible defaults)
- It adds new BLE characteristics that older apps can ignore or that do not break App usage
- It improves performance without altering outputs

## Version sources of truth

### Firmware
Firmware version is defined in `firmware/src/Version.h`:
- `FIRMWARE_VERSION_MAJOR`
- `FIRMWARE_VERSION_MINOR`
- `FIRMWARE_VERSION_PATCH`
- `FIRMWARE_VERSION_STRING`

## Release channels
- All updates happen via the [Eidon Sym](https://sym.eidon.ai)
- Direct github updates are possible but not recommended