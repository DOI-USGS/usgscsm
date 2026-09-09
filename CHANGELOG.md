# Changelog

All changes that impact users of this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

<!---
This document is intended for users of the applications and API. Changes to things
like tests should not be noted in this document.

When updating this file for a PR, add an entry for your change under Unreleased
and one of the following headings:
 - Added - for new features.
 - Changed - for changes in existing functionality.
 - Deprecated - for soon-to-be removed features.
 - Removed - for now removed features.
 - Fixed - for any bug fixes.
 - Security - in case of vulnerabilities.

If the heading does not yet exist under Unreleased, then add it as a 3rd heading,
with three #.


When preparing for a public release candidate add a new 2nd heading, with two #, under
Unreleased with the version number and the release date, in year-month-day
format. Then, add a link for the new version at the bottom of this document and
update the Unreleased link so that it compares against the latest release tag.


When preparing for a bug fix release create a new 2nd heading above the Fixed
heading to indicate that only the bug fixes and security fixes are in the bug fix
release.
-->

## [Unreleased]

### Added
- A `CASSIS` distortion type implementing the TGO CaSSIS rational ratio-of-quadratics distortion model, matching ISIS `TgoCassisDistortionMap`, with the json-name, ALE integer-enum, and coefficient-extraction dispatch wired in. Pairs with the ALE TGO CaSSIS driver. [#512](https://github.com/DOI-USGS/usgscsm/pull/512)

### Changed
- Resolve the distortion type from an ISD using usgscsm's own `DistortionType` enum rather than mapping through ale's enum, and throw on an unknown or missing distortion model instead of silently defaulting to `TRANSVERSE`. [#526](https://github.com/DOI-USGS/usgscsm/pull/526)

### Fixed
- `getReferenceDateAndTime` no longer truncates the ephemeris time to whole seconds, which gave framelets of a framing instrument such as JunoCam an identical reference time and serial number, so jigsaw rejected them as duplicates. The time is kept as a `double`, and the calendar string is now emitted to microseconds, the resolution many satellites use since several scan lines can be acquired within a millisecond. [#534](https://github.com/DOI-USGS/usgscsm/pull/534)
- The frame and SAR sensor models no longer emit a log message when `constructStateFromIsd` cannot build a model from the given ISD (the frame model logged at error level, the SAR model at info level). All four sensor models already throw a `csm::Error` in this case, and the caller catches it; the extra log message was redundant. During the normal probing done when loading a camera, where each model type is tried in turn until one succeeds, this produced a spurious error on every load of a valid non-frame camera. The linescan and pushframe models were already silent here, and this aligns the frame and SAR models with them. [#533](https://github.com/DOI-USGS/usgscsm/pull/533)
- Read `m_detectorSampleSumming` and `m_detectorLineSumming` from the model state as `double` rather than `int`. Bug fix. [#523](https://github.com/DOI-USGS/usgscsm/pull/523)
- `constructModelFromISD` now attempts the projected sensor model only when the ISD declares a projection (has a `geotransform`). A frame or unprojected linescan ISD is built directly, without the projected attempt that would always fail on the missing `geotransform` and log a misleading error on a normal load. A genuine failure of a projected ISD is still logged as an error. [#525](https://github.com/DOI-USGS/usgscsm/pull/525)
- `applyDistortion` and `removeDistortion` now throw on a distortion type they do not handle instead of silently returning the point undistorted. This covers the `LUNARORBITER` type, which is declared and emitted by ALE but was never implemented here, and any type from a model state that this build does not recognize. Previously such a model was silently treated as distortion-free, producing wrong geometry with no warning. [#521](https://github.com/DOI-USGS/usgscsm/pull/521)
- Changed the default log level from `INFO` to `ERROR` so high-volume callers are not stalled by per-call logging in `groundToImage`/`imageToGround`. The level is still overridable with the `USGSCSM_LOG_LEVEL` environment variable. [#514](https://github.com/DOI-USGS/usgscsm/pull/514)
- The Windows build now produces and installs the `usgscsm` import library alongside `usgscsm.dll`, and `usgscsm_cam_test` links against it. Enabled `WINDOWS_EXPORT_ALL_SYMBOLS` on the `usgscsm` target, and added RUNTIME and ARCHIVE destinations to its install rule, which the previous `LIBRARY`-only tagging omitted on Windows. Unix behavior is unchanged. [#518](https://github.com/DOI-USGS/usgscsm/pull/518)

## [2.1.0] - 2026-06-09

### Added
- `isUsgsCsmIsd()` and `isUsgsCsmState()` quick string-scan helpers in `UsgsAstroPluginSupport` to identify ISD vs model state without full JSON parsing or model construction. [#502](https://github.com/DOI-USGS/usgscsm/pull/502)
- Added the `KPLOSHADOWCAM` distortion type for the KPLO ShadowCam imager. [#505](https://github.com/DOI-USGS/usgscsm/pull/505)

### Fixed
- Fixed `ephemTimeToCalendarTime` to use the correct J2000 epoch (noon TT, not midnight UTC) and account for leap seconds via a hardcoded table. Previously off by 12 hours; now matches NAIF `et2utc` to the millisecond. [#506](https://github.com/DOI-USGS/usgscsm/pull/506)
- Fix a bug in the Frame Sensor Model, the ephemeris time was rounded to it. [#497](https://github.com/DOI-USGS/usgscsm/pull/497)
- Removed boundary checks for Frame Sensor Model getSensorPosition [#492](https://github.com/DOI-USGS/usgscsm/pull/492)
- Fixed CAHVOR model optical shifts by removing tolerance check [#488](https://github.com/DOI-USGS/usgscsm/issues/488)

## [2.0.1] - 2024-01-23

### Changed
- Updated USGSCSM build process to internally build dependencies into the library with no linking. [#445](https://github.com/DOI-USGS/usgscsm/pull/445)

## [2.0.0] - 2024-01-05

### Added
- Added support for the radial and tangential distortion model [#466](https://github.com/DOI-USGS/usgscsm/pull/466)

### Changed
- Made FrameSensor members public [#455](https://github.com/DOI-USGS/usgscsm/pull/455)
- Updated installation location [#467](https://github.com/DOI-USGS/usgscsm/pull/467) 
- Updated ALE submodule [#470](https://github.com/DOI-USGS/usgscsm/pull/470)

### Fixed
- Fixed issue with radial distortion computation [#464](https://github.com/DOI-USGS/usgscsm/pull/464)
