.. _isd_generation:

Creating and testing an ISD
===

This tutorial assumes that the user has created a conda environment with ale, usgscsm,
and csmapi installed.

Generating an ISD with ALE
-----------------
ISD generation is the first step in processing an image with the models provided
by USGSCSM.  ISDs can be easily generated with the use of
`ALE <https://github.com/USGS-Astrogeology/ale>`_, which generates ISDs through
the use of the "load" and "loads" functions  After installing the library
(as described on the repository), one can generate an ISD using the image's label
in the following fashion::

    import ale

    image_label_path = "/path/to/my/image.lbl"
    isd_string = ale.loads(image_label_path)

Under some circumstances, ALE fails to identify a set of kernels to properly
generate an ISD, and users will receive the message

    "No Such Driver for Label."

Under these circumstances, the user must explicitly pass the desired kernel set
to the loads function.  This kernel set can be generated from a spiceinit'd cub
and passed to ale as follows:

    kernels = ale.util.generate_kernels_from_cube(spice_initted_cub, expand=True)
    isd = ale.loads(img_file, props={'kernels': kernels})

After generating the ISD in Python, it is necessary to save it to disc. This is
accomplished via Python builtin functions such as:

    with open('out.json', 'w') as f:
        f.write(isd)

Generating an ISD with Pfeffernusse
-------------
Alternatively, ISDs can be generated via web service through the API exposed by
`Pfeffernusse <https://app.swaggerhub.com/apis/USGS-Astro/pfeffernusse2/0.1.4-oas3>`_.
While the web service is not yet publicly hosted, a public version of the service
is under development.  Individuals or organizations may choose to host their own
service, and it can be accessed as follows::

    curl -X POST "http://<servername>:<port>/v1/pds/" -H \
    "accept: application/json" -H "Content-Type: application/json" \
    -d @EN1040199536M_tmp.json

where EN1040199536M_tmp.json is a file containing a JSON representation of an
image label.

Testing the ISD
---------------
USGSCSM includes a test utility that can be used to verify the integrity of an
ISD by loading a camera model and performing basic tests.  This utility is
located in the USGSCSM build directory, and requires only the path to the ISD
from the previous step.

    ./usgscsm_cam_test --model /Path/to/isd.json

A successful test will result in output similar to the following:

    Detected CSM plugin: UsgsAstroPluginCSM
    Number of models for this plugin: 4
    Loaded a CSM model of type USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL from ISD file ...
