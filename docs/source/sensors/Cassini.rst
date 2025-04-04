Cassini Imaging Subsystem
=========================
The Cassini Imaging Subsystem (ISS) comprises two instruments -- a narrow angle camera
and a wide angle camera.  The narrow angle camera contains 24 filters, and the
wide angle camera contains 18 filters.

Processing Cassini ISS Images
-----------------------------
.. code-block:: python

  import ale, json, os
  from ale.drivers.cassini_drivers import CassiniIssPds3LabelNaifSpiceDriver
  from ale.drivers import JsonEncoder

  # Use the images to generate ISDs and create CSM cameras
  # Assume images are in current directory

  nac_stereo_1 = 'N1702360370_1.LBL'
  nac_stereo_2 = 'N1702360308_1.LBL'

  def generate_isd(filename):
      driver = CassiniIssPds3LabelNaifSpiceDriver(filename)

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
  nac1_dict = generate_isd(nac_stereo_1)
  nac2_dict = generate_isd(nac_stereo_2)

  # Construct a camera
  camera1 = csm.create_csm(nac_stereo_1)
  camera2 = csm.create_csm(nac_stereo_2)
