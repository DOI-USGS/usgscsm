MiniRF
======
MiniRF is the Lunar Reconnaissance Orbiter's Miniature Radio Frequency instrument.
This instrument features two wavelength bands, and has a resolution of 30 meters
per pixel.


Processing MiniRF Images
------------------------

LRO Narrow Angle Camera
=======================
LRO Narrow Angle Camera (NAC) is the Lunar Reconnaissance Orbiter's Narrow Angle Camera.  It is a
panchromatic linescan imager with a resolution of .5 meters per pixel over a 5km
swath.

Processing LRO NAC Images
-------------------------

LRO Wide Angle Camera
=====================
LRO Wide Angle Camera (WAC) is the Lunar Reconnaissance Orbiter's Wide Angle Camera.  This is a
pushframe camera with a resolution of 100 meters per pixel over a 65km swath.
This instrument captures data in 7 bands.

Processing LRO WAC Images
-------------------------
LRO WAC uses a pushframe camera model, and an overview of the processing steps
for this sensor is as follows:

1. Use ISIS to ingest, calibrate, and spiceinit your cubes
2. (Optional) photometrically correct the images
3. Use framestitch to re-combine your even and odd frames
4. Use ale to generate ISDs for your images, the framestitch application will
   strip the ISIS camera model info off the cubes, so you'll need to pass ALE
   your un-stitched images. It doesn't matter if you use even or odd´.
5. Load your generated ISDs and stitched cubes into your CSM SET


This example uses M119929852ME.IMG, which can be easily downloaded from the
terminal via

    $ curl http://pds.lroc.asu.edu/data/LRO-L-LROC-2-EDR-V1.0/LROLRC_0002/DATA/MAP/2010035/WAC/M119929852ME.IMG -o M119929852ME.IMG


Ingesting, calibrating, and spiceinitting cubes
-----------------------------------------------
Ingesting, calibrating, and spiceinitting files results in an ISIS´-formatted
image file that has been dark-field, flat-field, radiometric, and temperature
corrected and initialized with spice kernels.  These processes can be completed
in a terminal with the following ISIS commands:

    $ lrowac2isis from= M119929852ME.IMG to = M119929852ME.cub
    $ lrowaccal from= M119929852ME.vis.even.cub to= M119929852ME.vis.even.cal.cub
    $ lrowaccal from= M119929852ME.vis.odd.cub to= M119929852ME.vis.odd.cal.cub
    $ spiceinit from= M119929852ME.vis.even.cal.cub
    $ spiceinit from= M119929852ME.vis.odd.cal.cub

Photometrically correcting cubes (optional)
-------------------------------------------
Further research is necessary before ASC developers can recommend specific
parameters for photometric correction.  While the following call is used to
perform the correction, the determination of the specific values used in the
input PVL is left to the user.

    $ photomet from= M119929852ME.vis.even.cal.cub to= M119929852ME.vis.even.cal.cor.cub
    $ photomet from= M119929852ME.vis.odd.cal.cub to= M119929852ME.vis.odd.cal.cor.cub


Using framestitch to stitch even and odd frames together
---------------------------------------------------------
Framestitch is responsible for combining the even and odd framelets into a single
image.  Note that the resulting, stitched image will not include camera model
information.

    $ framestitch even= M119929852ME.vis.even.cal.cor.cub odd= M119929852ME.vis.odd.cal.cub to= M119929852ME.vis.stitched.cal.cor.cub


Generating ISDs
---------------
For information on generating an ISD, review the section on :ref:`isd_generation`.
