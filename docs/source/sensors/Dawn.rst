Dawn FC
=======
The Dawn Framing Camera is a frame camera with a resolution of up to 25 meters
per pixel.  This instrument contains a clear filter and 7 band-pass filters.

Processing Dawn FC Images
-------------------------

.. code-block:: python

  import ale, json, os
  from ale.drivers.cassini_drivers import DawnFcPds3NaifSpiceDriver
  from ale.drivers import JsonEncoder

  # Use the images to generate ISDs and create CSM cameras
  # Assume images are in current directory

  dawn_xmpl = 'DAWNFC_1.LBL'

  def generate_isd(filename):
      driver = DawnFcPds3NaifSpiceDriver(filename)

      # SPICE kernels are furnished inside this with
      with driver as d:
          # this is the information for the ISD in a python dict
          aledict = d.to_dict()

          # Export python dictionary ISD to external json file to be used by CSM
          alelabel = os.path.splitext(filename)[0]+".json"
          with open (alelabel, "w") as file:
            json.dump(aledict, file, cls=JsonEncoder)
          return aledict

  # Generate ISD and export to a json file
  xmpl_dict = generate_isd(dawn_xmpl)

  # Construct a camera
  cam_xmpl = csm.create_csm(dawn_xmpl)
