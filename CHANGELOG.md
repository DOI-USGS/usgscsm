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
- `isUsgsCsmIsd()` and `isUsgsCsmState()` quick string-scan helpers in `UsgsAstroPluginSupport` to identify ISD vs model state without full JSON parsing or model construction. [#502](https://github.com/DOI-USGS/usgscsm/pull/502)
- Added `KPLOSHADOWCAM` distortion type for the KPLO ShadowCam imager. Single-coefficient Y-only cubic distortion (closed-form distorted-to-undistorted: `uy = dy * (1 + dk1 * dy^2)`; iterative inverse via fixed-point on `yt = uy / (1 + dk1 * yt^2)`). Includes string parser, JSON coefficient parser, and updated int-mapper for `ale::DistortionType::KPLOSHADOWCAM` in `Utilities.cpp`. Matches the `kplo_shadowcam` distortion emitted by the corresponding ALE driver. Ported from `isis/src/kplo/objs/ShadowCamCamera/ShadowCamDistortionMap.cpp`.

### Fixed
- Fix a bug in the Frame Sensor Model, the ephemeris time was rounded to it. [#497](https://github.com/DOI-USGS/usgscsm/pull/497)
- Removed boundary checks for Frame Sensor Model getSensorPosition [#492](https://github.com/DOI-USGS/usgscsm/pull/492)
- Fixed CAHVOR model optical shifts by removing tolerance check [#488](https://github.com/DOI-USGS/usgscsm/issues/488)
- Fixed `getDistortionModel(int, ...)` falling through to the default `TRANSVERSE` case for any `ale::DistortionType` enum value beyond `RADTAN`. Previously, every distortion type added to ALE after RADTAN had no matching case in the int-to-enum mapper used by `UsgsAstroLsSensorModel::constructStateFromIsd`, so the corresponding `m_distortionType` was silently overwritten with `TRANSVERSE` (default fallback). KPLOSHADOWCAM is the first type to expose this; the mapper now explicitly handles it and matches the existing string-key mapper.

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
