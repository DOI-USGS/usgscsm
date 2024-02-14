Kaguya TC
=========
The Kaguya Terrain Camera is a dual-band, stereo, linescan camera with a spatial
resolution of 20 meters per pixel.  This set of sensors uses two telescopic
imaging instruments, one pointed forward, and one pointed backward. Together,
these instruments provide data necessary to create stereoscopic imagery.

Processing Kaguya TC Images
---------------------------
Begin by generating an ISD as described in the section on :ref:`isd_generation`.

.. code-block:: python

    import json
    import os
    import csmapi
    import ale
    import pvl
    from ale.drivers import JsonEncoder
    from ale.drivers.kaguya_drivers import KaguyaTcPds3NaifSpiceDriver

    image = "images/TC2W2B0_01_00366S490E1640.img"
    alelabel = os.path.splitext(image)[0]+".json"

    label = pvl.load(image)
    driver = KaguyaTcPds3NaifSpiceDriver(image)

    with driver as d:
        aledict = d.to_dict()

    json.dump(aledict, open(alelabel, "w"), cls=JsonEncoder)

    nlines, nsamples = aledict["image_lines"], aledict["image_samples"]

    model="USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL"
    plugin = csmapi.Plugin.getList()[0]
    isd = csmapi.Isd(alelabel)
    camera = plugin.constructModelFromISD(isd, model)

Kaguya MI
=========
The Kaguya Multiband Imagery platform comprises two different instruments -- a
visible light sensor (VIS) and a near infrared sensor (NIR).  The visible light
sensor has a spatial resolution of 20 meters per pixel, and collects 5 bands of
data via a bandpass filter.  The near infrared sensor is responsible for
collecting 4 bands of data at a resolution of 62 meters per pixel.

Processing Kaguya MI Images
---------------------------
