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