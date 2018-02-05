import os
import json
import pytest

from cycsm import isd
import cycsm.csm as csm
import usgscam as cam

data_path = os.path.dirname(__file__)

class TestCassiniNAC:
    @pytest.fixture
    def cassini_model(self):
        path = os.path.join(data_path,'cassini_nac.json')
        with open(path, 'r') as f:
            csm_isd = isd.Isd.load(f)
        plugin = cam.genericframe.Plugin()
        return plugin.from_isd(csm_isd, plugin.modelname(1))

    @pytest.mark.parametrize('image, ground',[
                              ((512, 512, 0), (232745.39404384792, 108032.64063985727, 1416.4229696467519)),
                              ((1024, 1024, 0), (208934.79595478065, 147272.0343555567, 21924.160632191226)),
                              ((0, 0, 0), (247048.74337707553, 68020.01669191488, -13280.94752286654))
    ])
    def test_image_to_ground(self, cassini_model, image, ground):
        gx, gy, gz = ground
        x, y, z = cassini_model.imageToGround(*image)

        assert x == pytest.approx(gx, rel=1)
        assert y == pytest.approx(gy, rel=1)
        assert z == pytest.approx(gz, rel=1)

    @pytest.mark.parametrize('image, ground',[
                              ((512, 512, 0), (232745.39404384792, 108032.64063985727, 1416.4229696467519)),
                              ((1024, 1024, 0), (208934.79595478065, 147272.0343555567, 21924.160632191226)),
                              ((0, 0, 0), (247048.74337707553, 68020.01669191488, -13280.94752286654))
    ])
    def test_ground_to_image(self, cassini_model, image, ground):
        y, x = cassini_model.groundToImage(*ground)
        ix, iy, _ = image
        assert ix == pytest.approx(x, rel=11)
        assert iy == pytest.approx(y, rel=11)


class TestMdisWac:
    @pytest.fixture
    def model(self):
        csm_isd = isd.Isd()
        with open(os.path.join(data_path,'CW1071364100B_IU_5.json'), 'r') as f:
            d = json.load(f)
        for k, v in d.items():
            csm_isd.addparam(k, v)

        plugin = cam.genericframe.Plugin()
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
        return cam.genericframe.Plugin()

    @pytest.fixture
    def model(self):
        csm_isd = isd.Isd()
        with open(os.path.join(data_path,'EN1007907102M.json'), 'r') as f:
            d = json.load(f)
        for k, v in d.items():
            csm_isd.addparam(k, v)

        plugin = cam.genericframe.Plugin()
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


    @pytest.mark.parametrize('time',[
                             (418855170.49264956),
    ])
    def test_sensorposition_time(self, model, time):
        x, y, z = model.sensor_time_position(time)

    def test_sensorposition_coordinate(self, model):
        x, y, z = model.sensor_coordinate_position(512.0, 512.0)
        assert x == pytest.approx(1728181.03)
        assert y == pytest.approx(-2088202.59)
        assert z == pytest.approx(2082707.61)

    @pytest.mark.parametrize('time',[
                             (418855170.49264956),
    ])
    def test_sensorvelocity_time(self, model, time):
        x, y, z = model.sensor_time_velocity(time)
        assert x == 0
        assert y == 0
        assert z == 0

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

    def test_imagerange(self, model):
        min_pt, max_pt = model.imagerange
        assert min_pt == (1.0, 9.0)
        assert max_pt == (1024, 1024)

    def test_heightrange(self, model):
        assert model.heightrange == (-100,100)

    #def test_version(self, model):
    #    assert model.version.version.decode() == "0.1.0"

    def test_modelname(self, model):
        assert model.name == "USGS_ASTRO_FRAME_SENSOR_MODEL"

    def test_sensortype(self, model):
        assert model.sensortype == "EO"

    def test_sensormode(self, model):
        assert model.sensormode == "FRAME"

    @pytest.mark.parametrize('image, ground, truth',[
        ((512, 512, 0), (1129256.7251961431, -1599248.4677779, 1455285.5207515),
        (-0.022112677416771476, -0.022440933181201217)),
        ((100, 100, 0), (1115975.1523941057, -1603416.1960299, 1460934.0579053),
        (-0.021533852098798434, -0.022313040556070973))
    ])
    def test_compute_sensor_partials_with_image(self, model, image, ground, truth):
        partials = model.compute_sensor_partials(0, *ground, line=image[0], sample=image[1])
        assert pytest.approx(partials,truth)

    @pytest.mark.parametrize('ground, truth',[
        ((1129256.7251961431, -1599248.4677779, 1455285.5207515),
        (-0.022112677416771476, -0.022440933181201217)),
        ((1115975.1523941057, -1603416.1960299, 1460934.0579053),
        (-0.021533852098798434, -0.022313040556070973))
    ])
    def test_compute_sensor_partials_without_image(self, model, ground, truth):
        partials = model.compute_sensor_partials(0, *ground)
        assert pytest.approx(partials, 0.1) == truth

    @pytest.mark.parametrize('ground, truth',[
        ((1129256.7251961431, -1599248.4677779, 1455285.5207515),
         [(-0.022112677416771476, -0.022440933181201217),
         (0.011881799159482398, -0.032259811189305765),
         (0.03036786720906548, -0.0037186004832392427),
         (-491255.371471137, 1.759792667144211e-08),
         (-18407.9265443473, -66608.06733288719),
         (46820.35586710303, -136243.26154801922)]),
        ((1115975.1523941057, -1603416.1960299, 1460934.0579053),
         [(-0.021533852098798434, -0.022313040556070973),
         (0.011931088289486524, -0.032481524362651726),
         (0.030504941456740653, -0.003355589999443964),
         (-524050.03508186026, -6640.755516575846),
         (-20179.090856298106, -67377.90738139131),
         (49348.36109812631, -145968.62944389175)])
    ])
    def test_compute_all_sensor_partials_without_image(self, model, ground, truth):
        partials = model.compute_all_sensor_partials(*ground)
        for i in range(len(partials)):
            assert pytest.approx(partials[i], 0.1) == truth[i]

    @pytest.mark.parametrize('image, ground, truth',[
        ((512, 512, 0), (1129256.7251961431, -1599248.4677779, 1455285.5207515),
        ((-0.022112677416771476, -0.022440933181201217),
         (0.011881799159482398, -0.032259811189305765),
         (0.03036786720906548, -0.0037186004832392427),
         (-491255.371471137, 1.759792667144211e-08),
         (-18407.9265443473, -66608.06733288719),
         (46820.35586710303, -136243.26154801922))),
        ((100, 100, 0), (1115975.1523941057, -1603416.1960299, 1460934.0579053),
        ((-0.021533852098798434, -0.022313040556070973),
         (0.011931088289486524, -0.032481524362651726),
         (0.030504941456740653, -0.003355589999443964),
         (-524050.03508186026, -6640.755516575846),
         (-20179.090856298106, -67377.90738139131),
         (49348.36109812631, -145968.62944389175)))
    ])
    def test_compute_all_sensor_partials_with_image(self, model, image, ground, truth):
        partials = model.compute_all_sensor_partials(*ground, line=image[0], sample=image[1])
        for i in range(len(partials)):
            assert pytest.approx(partials[i], 0.1) == truth[i]

    @pytest.mark.parametrize('ground, truth',[
        ((1129256.7251961431, -1599248.4677779, 1455285.5207515),
        [0.019424449430102384, -0.08345396655672399, -0.08357840280904191,
         -0.022145787236569233, 0.27363142654460765,0.23438267062135587]),
        ((1115975.1523941057, -1603416.1960299, 1460934.0579053),
        [0.017765091042620192,-0.07292865138581625,-0.07435297169065827,
        -0.02015836541725924, 0.2536340138413847,0.21760235526959235])
    ])
    def test_compute_ground_partials(self, model, ground, truth):
        partials = model.compute_ground_partials(*ground)
        assert pytest.approx(partials,truth)

    def test_get_set_parameter_type(self, model):
        ptype = model.get_parameter_type(0)
        assert ptype.name == 'REAL'
        model.set_parameter_type(0, 0)
        ptype = model.get_parameter_type(0)
        assert ptype.name == 'NONE'
