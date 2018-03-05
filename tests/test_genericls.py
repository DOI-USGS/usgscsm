import os
import json
import pytest

from cycsm import isd
import cycsm.csm as csm
import usgscam as cam

data_path = os.path.dirname(__file__)

@pytest.fixture
def hrsc_nadir_model():
    path = os.path.join(data_path, 'h0232_0000_nd2_keywords.lis')
    csm_isd = isd.Isd.read_socet_file(path)
    plugin = cam.genericls.Plugin()
    return plugin.from_isd(csm_isd, plugin.modelname(1))

@pytest.fixture
def hrsc_stereo_1_model():
    path = os.path.join(data_path, 'h0232_0000_s12_keywords.lis')
    csm_isd = isd.Isd.read_socet_file(path)
    plugin = cam.genericls.Plugin()
    return plugin.from_isd(csm_isd, plugin.modelname(1))

@pytest.fixture
def hrsc_stereo_2_model():
    path = os.path.join(data_path, 'h0232_0000_s22_keywords.lis')
    csm_isd = isd.Isd.read_socet_file(path)
    plugin = cam.genericls.Plugin()
    return plugin.from_isd(csm_isd, plugin.modelname(1))

class TestCTX:

    @pytest.mark.parametrize('image, ground',[
                              ((2500, 9216, 0), (-73589.5516508502, 562548.342040933, 2372508.44060771))
    ])
    def test_image_to_ground(self, ctx_model, image, ground):
        gx, gy, gz = ground
        x, y, z = ctx_model.imageToGround(*image)
        #TODO: Get this test up and running.
        #print(x, y, z)
        #assert False
        #assert x == pytest.approx(gx, rel=1)
        #assert y == pytest.approx(gy, rel=1)
        #assert z == pytest.approx(gz, rel=1)

    #@pytest.mark.parametrize('image, ground',[
    #                          ((512, 512, 0), (-73589.5516508502, 562548.342040933, 2372508.44060771)),
    #                          ((100, 100, 0), (-48020.2164819883, 539322.805489926, 2378549.41724731))
    #])
    #def test_ground_to_image(self, model, image, ground):
    #    y, x = model.groundToImage(*ground)
    #    ix, iy, _ = image
#
    #    assert x == pytest.approx(ix)
    #    assert y == pytest.approx(iy)

class TestHRSCNadir:

    @pytest.mark.parametrize('image, ground',[
          ((0.5, 0.5, 0), (985377.89225802, 3243484.7261571, 206009.0045894)),
          ((1.0, 1.0, 0), (985364.97668174, 3243488.0289798, 206018.66578845)),
          ((0.5, 5175.5, 0), (849592.95667544, 3281524.2988248, 208281.94748872)),
          ((25503.5, 0.5, 0), (995812.30843397, 3161653.5178064, 734845.5368816)),
          ((25503.5, 5175.5, 0), (817255.33536118, 3211512.2358805, 738854.37876771)),
          ((12751.5, 2587.5, 0), (914645.41695902, 3237864.2204448, 459626.50243137))
    ])
    def test_image_to_ground(self, hrsc_nadir_model, image, ground):
        gx, gy, gz = ground
        x, y, z = hrsc_nadir_model.imageToGround(*image)
        assert x == pytest.approx(gx, abs=20)
        assert y == pytest.approx(gy, abs=20)
        assert z == pytest.approx(gz, abs=20)

    @pytest.mark.parametrize('image, ground',[
          ((0.5, 0.5, 0), (985377.89225802, 3243484.7261571, 206009.0045894)),
          ((0.5, 5175.5, 0), (849592.95667544, 3281524.2988248, 208281.94748872)),
          ((25503.5, 0.5, 0), (995812.30843397, 3161653.5178064, 734845.5368816)),
          ((25503.5, 5175.5, 0), (817255.33536118, 3211512.2358805, 738854.37876771)),
          ((12751.5, 2587.5, 0), (914645.41695902, 3237864.2204448, 459626.50243137))
    ])
    def test_ground_to_image(self, hrsc_nadir_model, image, ground):
        y, x = hrsc_nadir_model.groundToImage(*ground)
        iy, ix, _ = image

        assert x == pytest.approx(ix, abs=0.5)
        assert y == pytest.approx(iy, abs=0.5)

class TestHRSCStereo1:

    @pytest.mark.parametrize('image, ground',[
          ((0.5, 0.5, 0), (979590.30174957, 3242179.3919958, 249088.83886381)),
          ((0.5, 2583.5, 0), (855179.96698611, 3277243.42159, 248427.66616907)),
          ((12503.5, 0.5, 0), (984596.8817312, 3152386.5706555, 787256.94481508)),
          ((12503.5, 2583.5, 0), (824462.92414568, 3197824.047306, 787981.15939371)),
          ((6251.5, 1291.5, 0), (913762.62459356, 3231516.1914323, 503425.73182682))
    ])
    def test_image_to_ground(self, hrsc_stereo_1_model, image, ground):
        gx, gy, gz = ground
        x, y, z = hrsc_stereo_1_model.imageToGround(*image)
        assert x == pytest.approx(gx, abs=100)
        assert y == pytest.approx(gy, abs=100)
        assert z == pytest.approx(gz, abs=100)

    @pytest.mark.parametrize('image, ground',[
          ((0.5, 0.5, 0), (979590.30174957, 3242179.3919958, 249088.83886381)),
          ((0.5, 2583.5, 0), (855179.96698611, 3277243.42159, 248427.66616907)),
          ((12503.5, 0.5, 0), (984596.8817312, 3152386.5706555, 787256.94481508)),
          ((12503.5, 2583.5, 0), (824462.92414568, 3197824.047306, 787981.15939371)),
          ((6251.5, 1291.5, 0), (913762.62459356, 3231516.1914323, 503425.73182682))
    ])
    def test_ground_to_image(self, hrsc_stereo_1_model, image, ground):
        y, x = hrsc_stereo_1_model.groundToImage(*ground)
        iy, ix, _ = image

        assert x == pytest.approx(ix, abs=0.5)
        assert y == pytest.approx(iy, abs=0.5)

class TestHRSCStereo2:

    @pytest.mark.parametrize('image, ground',[
          ((0.5, 0.5, 0), (994968.78141471, 3243624.373581, 150910.83677465)),
          ((0.5, 2583.5, 0), (840154.182443, 3286852.5036334, 156704.92657188)),
          ((14263.5, 0.5, 0), (1015683.4735728, 3170802.9252193, 665761.34487006)),
          ((14263.5, 2583.5, 0), (802816.11887483, 3229529.8393814, 674042.87178595)),
          ((7131.5, 1291.5, 0), (915544.01372502, 3243112.89855, 419540.13516932))
    ])
    def test_image_to_ground(self, hrsc_stereo_2_model, image, ground):
        gx, gy, gz = ground
        x, y, z = hrsc_stereo_2_model.imageToGround(*image)
        assert x == pytest.approx(gx, abs=100)
        assert y == pytest.approx(gy, abs=100)
        assert z == pytest.approx(gz, abs=100)

    @pytest.mark.parametrize('image, ground',[
          ((0.5, 0.5, 0), (994968.78141471, 3243624.373581, 150910.83677465)),
          ((0.5, 2583.5, 0), (840154.182443, 3286852.5036334, 156704.92657188)),
          ((14263.5, 0.5, 0), (1015683.4735728, 3170802.9252193, 665761.34487006)),
          ((14263.5, 2583.5, 0), (802816.11887483, 3229529.8393814, 674042.87178595)),
          ((7131.5, 1291.5, 0), (915544.01372502, 3243112.89855, 419540.13516932))
    ])
    def test_ground_to_image(self, hrsc_stereo_2_model, image, ground):
        y, x = hrsc_stereo_2_model.groundToImage(*ground)
        iy, ix, _ = image

        assert x == pytest.approx(ix, abs=0.5)
        assert y == pytest.approx(iy, abs=0.5)
