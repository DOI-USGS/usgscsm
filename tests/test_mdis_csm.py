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
        with open(os.path.join(data_path,'EN1007907102M.json'), 'r') as f:
            d = json.load(f)
        for k, v in d.items():
            csm_isd.addparam(k, v)
        return csm_isd

    def test_nmodels(self, plugin):
        assert plugin.nmodels == 1

    def test_check_isd_construction(self, plugin, i):
        assert plugin.check_isd_construction(i, plugin.modelname(1))

    def test_releasedate(self, plugin):
        assert plugin.releasedate == "20170425"

    def test_csmversion(self, plugin):
        assert plugin.csmversion.version.decode() == "3.0.1"

    def test_plugin_name(self, plugin):
        assert plugin.name == 'UsgsAstroFrameMdisPluginCSM'

    def test_modelname(self, plugin):
        assert plugin.modelname(1) == 'ISIS_MDISNAC_USGSAstro_1_Linux64_csm30.so'

    def test_modelfamily(self, plugin):
        assert plugin.modelfamily(1) == 'Raster'

    def test_modelname_from_state(self, plugin, i):
        state = {'model_name': 'ISIS_MDISNAC_USGSAstro_1_Linux64_csm30.so'}
        name = plugin.modelname_from_state(json.dumps(state))
        assert name == state['model_name']

    def test_modelname_from_state_bad_key(self, plugin):
        state = {'mname':'foo'}
        with pytest.raises(RuntimeError) as err:
            name = plugin.modelname_from_state(json.dumps(state))
        assert "'model_name'" in str(err.value)

    def test_modelname_from_state_bad_name(self, plugin):
        state = {'model_name':'foo'}
        with pytest.raises(RuntimeError) as err:
            name = plugin.modelname_from_state(json.dumps(state))
        assert "Sensor model not supported." in str(err.value)

    def test_can_model_be_constructed_from_state(self, plugin):
        name = 'ISIS_MDISNAC_USGSAstro_1_Linux64_csm30.so'
        with open(os.path.join(data_path,'nac_state.json'), 'r') as f:
            state = json.load(f)
        constructible = plugin.can_model_be_constructed_from_state(name, json.dumps(state))
        assert constructible == True

        name = 'foo'
        constructible = plugin.can_model_be_constructed_from_state(name, json.dumps(state))
        assert constructible == False

    def test_can_isd_be_converted_to_model_state(self, plugin, i):
        name = 'ISIS_MDISNAC_USGSAstro_1_Linux64_csm30.so'
        res = plugin.can_isd_be_converted_to_model_state(i, name)
        assert res == True

    def test_convert_isd_to_model_state(self, plugin, i):
        name = 'ISIS_MDISNAC_USGSAstro_1_Linux64_csm30.so'
        state = plugin.convert_isd_to_state(i, name)
        with open(os.path.join(data_path,'nac_state.json'), 'r') as f:
            truth = json.load(f)
        assert state == truth

    def test_convert_isd_to_model_state_bad(self, plugin, i):
        # Trash the isd and check error handling
        i.clear_params("focal_length")
        name = 'ISIS_MDISNAC_USGSAstro_1_Linux64_csm30.so'
        with pytest.raises(RuntimeError) as err:
            state = plugin.convert_isd_to_state(i, name)
        assert "Sensor model support data" in str(err.value)

    def test_construct_model_from_state(self, plugin):
        with open(os.path.join(data_path,'nac_state.json'), 'r') as f:
            state = json.load(f)
        camera = plugin.from_state(json.dumps(state))
        assert isinstance(camera, cam.mdis.MdisNacSensorModel)


class TestMdisWac:
    @pytest.fixture
    def model(self):
        return cam.mdis.MdisPlugin()

    @pytest.fixture
    def model(self):
        csm_isd = isd.Isd()
        with open(os.path.join(data_path,'CW1071364100B_IU_5.json'), 'r') as f:
            d = json.load(f)
        for k, v in d.items():
            csm_isd.addparam(k, v)

        plugin = cam.mdis.MdisPlugin()
        return plugin.from_isd(csm_isd, plugin.modelname(1))

    @pytest.mark.parametrize('image, ground',[
                              ((512, 512, 0), (-73589.5516508502, 562548.342040933, 2372508.44060771)),
                              ((100, 100, 0), (-48020.2164819883, 539322.805489926, 2378549.41724731))
    ])
    def test_image_to_ground(self, model, image, ground):
        gx, gy, gz = ground
        x, y, z = model.imageToGround(*image)
        assert x == pytest.approx(gx, rel=1)
        assert y == pytest.approx(gy, rel=1)
        assert z == pytest.approx(gz, rel=1)

    @pytest.mark.parametrize('image, ground',[
                              ((512, 512, 0), (-73589.5516508502, 562548.342040933, 2372508.44060771)),
                              ((100, 100, 0), (-48020.2164819883, 539322.805489926, 2378549.41724731))
    ])
    def test_ground_to_image(self, model, image, ground):
        y, x = model.groundToImage(*ground)
        ix, iy, _ = image

        assert x == pytest.approx(ix)
        assert y == pytest.approx(iy)

class TestMdisNac:
    @pytest.fixture
    def model(self):
        return cam.mdis.MdisPlugin()

    @pytest.fixture
    def model(self):
        csm_isd = isd.Isd()
        with open(os.path.join(data_path,'EN1007907102M.json'), 'r') as f:
            d = json.load(f)
        for k, v in d.items():
            csm_isd.addparam(k, v)

        plugin = cam.mdis.MdisPlugin()
        return plugin.from_isd(csm_isd, plugin.modelname(1))

    @pytest.mark.parametrize('image, ground',[
                              ((512, 512, 0), (1129256.7251961, -1599248.4677779, 1455285.5207515)),
                              ((100, 100, 0), (1115975.1523941, -1603416.1960299, 1460934.0579053))
    ])
    def test_image_to_ground(self, model, image, ground):
        gx, gy, gz = ground
        x, y, z = model.imageToGround(*image)
        assert x == pytest.approx(gx)
        assert y == pytest.approx(gy)
        assert z == pytest.approx(gz)



    @pytest.mark.parametrize('image, ground',[
                              ((512, 512, 0), (1129256.7251961431, -1599248.4677779, 1455285.5207515)),
                              ((100, 100, 0), (1115975.1523941057, -1603416.1960299, 1460934.0579053))
    ])
    def test_ground_to_image(self, model, image, ground):
        y, x = model.groundToImage(*ground)
        ix, iy, _ = image

        assert x == pytest.approx(ix)
        assert y == pytest.approx(iy)

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
