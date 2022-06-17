usgscsm_cam_test
================

This program is shipped with the USGSCSM library in the ``bin`` directory.
It can be used for performing several operations involving CSM camera
models, such as loading a camera model, whether in the original ISD format,
model state representation, or GXP .sup file exporting the model state, computing
projections from pixels in the camera to the ground and back, and
then verifying that the original pixels are obtained.

Example (load a camera model and save the model state)::

    usgscsm_cam_test --model input.json --output-model-state out_state.json

Example (perform per-pixel operations)::

    usgscsm_cam_test --model camera.json --sample-rate 100 \
       --height-above-datum 320.3 --subpixel-offset 0.57

Example (modify a GXP .sup with new model state)::

   usgscsm_cam_test --model camera.json --modify-sup-file gxp_file.sup

Command line options
~~~~~~~~~~~~~~~~~~~~

--model <string (default: "")>
    Input CSM model (in ISD, model state, or GXP .sup file format).

--output-model-state <string (default: "")>
    If specified, save the model state to this file.

--sample-rate <integer (default: 0)>
    If positive, select pixels in the camera at the intersection of
    every one out of this many rows and columns, and perform projections
    to the ground and back.

--subpixel-offset <double (default: 0.0)>
    Add this value to every pixel row and column to
    be sampled.

--height-above-datum <double (default: 0.0)>
    Let the ground be obtained from the datum for this camera by
    adding to its radii this value (the units are meters).

--desired-precision <double (default: 0.001)>
    Use this value for operations (ground-to-image and image-to-ground)
    which need a precision value. Measured in pixels.

--modify-sup-file (default: "")>
    Input GXP .sup file to be modified by inputted CSM model. This will override
    the existing .sup file's SENSOR_STATE.

--help <no value>
    Print the usage message.
