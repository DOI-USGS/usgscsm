#!/usr/bin/env python
"""
Verify ephemTimeToCalendarTime against NAIF SPICE.

Usage:
  conda activate isis_dev  # or any env with spiceypy
  python verify_ephemtime.py

Requires a leapseconds kernel (naif0012.tls) loadable via ISISDATA
or a direct path.
"""
import os
import ctypes
import spiceypy as spice

# Load leapseconds kernel
lsk = None
for root in [os.environ.get('ISISDATA', ''),
             os.environ.get('ALESPICEROOT', ''),
             os.path.expanduser('~/projects/isis3data')]:
    candidate = os.path.join(root, 'base/kernels/lsk/naif0012.tls')
    if os.path.isfile(candidate):
        lsk = candidate
        break
if lsk is None:
    raise FileNotFoundError("Cannot find naif0012.tls. Set ISISDATA.")
spice.furnsh(lsk)

# Load the usgscsm shared library
lib = None
for candidate in ['../build/libusgscsm.dylib',
                   '../build/libusgscsm.so',
                   '../install/lib/libusgscsm.dylib',
                   '../install/lib/libusgscsm.so']:
    if os.path.isfile(candidate):
        lib = ctypes.CDLL(candidate)
        break
if lib is None:
    raise FileNotFoundError("Cannot find libusgscsm.dylib/so. Build first.")

# Bind ephemTimeToCalendarTime (C++ returns std::string, use C wrapper if available)
# Since it returns std::string, we'll just compute expected values and compare
# via the test binary output instead.

# Test a range of ephemeris times spanning different leap-second eras
test_ets = [
    (-883656069.0, "1972-01-01 (first leap second era)"),
    (-631152000.0, "1980-01-01"),
    (-378691200.0, "1988-01-01"),
    (-126230400.0, "1996-01-01"),
    (0.0,          "J2000 epoch"),
    (189388800.0,  "2006-01-01"),
    (284040000.0,  "2009-01-01"),
    (378691200.0,  "2012-01-01"),
    (504921600.0,  "2016-01-01"),
    (536457600.0,  "2017-01-01 (last leap second)"),
    (725846400.0,  "2023-01-01"),
    (297088762.6,  "MRO CTX observation"),
    (299622941.6,  "Chandrayaan-1 observation"),
]

print(f"{'ET':>15s}  {'NAIF (spiceypy)':22s}  {'Description'}")
print("-" * 70)

for et, desc in test_ets:
    naif_utc = spice.et2utc(et, 'ISOC', 3) + 'Z'
    print(f"{et:>15.1f}  {naif_utc:22s}  {desc}")

print()
print("Compare against: cd tests && ../build/tests/runCSMCameraModelTests "
      "--gtest_filter='UtilitiesTests.*'")
print("Or add more ET values to the unit test and verify they match above.")
