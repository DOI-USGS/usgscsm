import os
import json
import pytest

from cycsm import isd
import cycsm.csm as csm
import usgscam as cam


data_path = os.path.dirname(__file__)
class TestPlugin:

    @pytest.fixture
    def plugin(self):
        return cam.mdis.MdisPlugin()

    @pytest.fixture
    def i(self):
        csm_isd = isd.Isd()
        with open(os.path.join(data_path,'isd.json'), 'r') as f:
            d = json.load(f)
        for k, v in d.items():
            csm_isd.addparam(k, v)
        return csm_isd

    def test_mdis_plugin(self, plugin):
        assert plugin.name == 'UsgsAstroFrameMdisPluginCSM'
        assert plugin.nmodels == 1
        assert plugin.modelname(1) == 'ISIS_MDISNAC_USGSAstro_1_Linux64_csm30.so'

    def test_check_isd_construction(self, plugin, i):
        assert plugin.check_isd_construction(i, plugin.modelname(1))



class TestMdisNac:
    @pytest.fixture
    def model(self):
        return cam.mdis.MdisPlugin()

    @pytest.fixture
    def model(self):
        csm_isd = isd.Isd()
        with open(os.path.join(data_path,'isd.json'), 'r') as f:
            d = json.load(f)
        for k, v in d.items():
            csm_isd.addparam(k, v)

        plugin = cam.mdis.MdisPlugin()
        return plugin.from_isd(csm_isd, plugin.modelname(1))

    @pytest.mark.parametrize('image, ground',[
                              ((512, 512, 0), (1129.25*1000, -1599.26*1000, 1455.28*1000)),
                              ((100, 100, 0), (1115.95*1000, -1603.44*1000, 1460.93*1000))
    ])
    def test_image_to_ground(self, model, image, ground):
        gx, gy, gz = ground
        x, y, z = model.imageToGround(*image)
        assert x == pytest.approx(gx, rel=1)
        assert y == pytest.approx(gy, rel=1)
        assert z == pytest.approx(gz, rel=1)

    @pytest.mark.parametrize('image, ground',[
                              ((512, 512, 0), (1129.25*1000, -1599.26*1000, 1455.28*1000)),
                              ((100, 100, 0), (1115.95*1000, -1603.44*1000, 1460.93*1000))
    ])
    def test_ground_to_image(self, model, image, ground):
        y, x = model.groundToImage(*ground)
        ix, iy, _ = image

        assert x == ix
        assert y == iy

    def test_imagestart(self, model):
        assert model.imagestart == (1.0,9.0) # From the ik kernel

    def test_imagesize(self, model):
        assert model.imagesize == (1024,1024)

    def test_illumination_direction(self, model):
        northpole = [0,0,2439.4 * 1000]
        x, y, z = model.illumination_direction(northpole)
        assert x == pytest.approx(31648725087.588726)
        assert y == pytest.approx(60633907522.72863)
        assert z == pytest.approx(2439.4*1000 - -38729485.77334732)

    def test_heightrange(self, model):
        with pytest.raises(NotImplementedError) as e:
            model.heightrange()

    def test_imagerange(self, model):
        with pytest.raises(NotImplementedError) as e:
            model.imagerange()

    def test_sensorposition_time(self, model):
        with pytest.raises(NotImplementedError) as e:
            model.sensor_time_position(12345.0)

    def test_sensorposition_coordinate(self, model):
        x, y, z = model.sensor_coordinate_position(512.0, 512.0)
        assert x == pytest.approx(1728181.03)
        assert y == pytest.approx(-2088202.59)
        assert z == pytest.approx(2082707.61)

    def test_sensorvelocity_time(self, model):
        with pytest.raises(NotImplementedError) as e:
            model.sensor_time_velocity(12345.0)

    def test_sensorvelcity_coordinate(self, model):
        x, y, z = model.sensor_coordinate_velocity(512.0, 512.0)
        assert x == 0
        assert y == 0
        assert z == 0

    def test_image_to_proximate_imaging_locus(self, model):
        point, direction = model.image_to_proximate_imaging_locus(512.0, 512.0, 0, 0, 0)
        assert direction['x'] == pytest.approx(-0.6015027)
        assert direction['y'] == pytest.approx(0.4910591)
        assert direction['z'] == pytest.approx(-0.630123)

        assert point['x'] == pytest.approx(1728181.03)
        assert point['y'] == pytest.approx(-2088202.59)
        assert point['z'] == pytest.approx(2082707.61)

    def test_image_to_remote_imaging_locus(self, model):
        point, direction = model.image_to_remote_imaging_locus(512, 512.0)
        assert direction['x'] == pytest.approx(-0.6015027)
        assert direction['y'] == pytest.approx(0.4910591)
        assert direction['z'] == pytest.approx(-0.630123)

        assert point['x'] == pytest.approx(1728181.03)
        assert point['y'] == pytest.approx(-2088202.59)
        assert point['z'] == pytest.approx(2082707.61)
