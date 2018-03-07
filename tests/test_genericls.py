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
          # ((0.5, 0.5, 0), (985377.89225802, 3243484.7261571, 206009.0045894)),
          # ((0.5, 5175.5, 0), (849592.95667544, 3281524.2988248, 208281.94748872)),
          # ((25503.5, 0.5, 0), (995812.30843397, 3161653.5178064, 734845.5368816)),
          # ((25503.5, 5175.5, 0), (817255.33536118, 3211512.2358805, 738854.37876771)),
          # ((12751.5, 2587.5, 0), (914645.41695902, 3237864.2204448, 459626.50243137))
            ((12751.5, 2587.5, 0), (914645.4170054727, 3237864.2203280977, 459626.5031521894)),
            ((0.5, 0.5, 0), (985377.8921162762, 3243484.7262333957, 206009.00407163633)),
            ((0.5, 5175.5, 0), (849592.9565413876, 3281524.298893377, 208281.94696064907)),
            ((25503.5, 0.5, 0), (995811.943316327, 3161654.168981308, 734843.2570738876)),
            ((25503.5, 5175.5, 0), (817254.9608542051, 3211512.854057241, 738852.1327129134))
    ])
    def test_image_to_ground(self, hrsc_nadir_model, image, ground):
        gx, gy, gz = ground
        x, y, z = hrsc_nadir_model.imageToGround(*image)
        assert x == pytest.approx(gx, abs=100)
        assert y == pytest.approx(gy, abs=100)
        assert z == pytest.approx(gz, abs=100)

    @pytest.mark.parametrize('image, ground',[
          # ((0.5, 0.5, 0), (985377.89225802, 3243484.7261571, 206009.0045894)),
          # ((0.5, 5175.5, 0), (849592.95667544, 3281524.2988248, 208281.94748872)),
          # ((25503.5, 0.5, 0), (995812.30843397, 3161653.5178064, 734845.5368816)),
          # ((25503.5, 5175.5, 0), (817255.33536118, 3211512.2358805, 738854.37876771)),
          # ((12751.5, 2587.5, 0), (914645.41695902, 3237864.2204448, 459626.50243137))
            ((12751.5, 2587.5, 0), (914645.4170054727, 3237864.2203280977, 459626.5031521894)),
            ((0.5, 0.5, 0), (985377.8921162762, 3243484.7262333957, 206009.00407163633)),
            ((0.5, 5175.5, 0), (849592.9565413876, 3281524.298893377, 208281.94696064907)),
            ((25503.5, 0.5, 0), (995811.943316327, 3161654.168981308, 734843.2570738876)),
            ((25503.5, 5175.5, 0), (817254.9608542051, 3211512.854057241, 738852.1327129134))
    ])
    def test_ground_to_image(self, hrsc_nadir_model, image, ground):
        y, x = hrsc_nadir_model.groundToImage(*ground)
        iy, ix, _ = image

        assert x == pytest.approx(ix, abs=0.5)
        assert y == pytest.approx(iy, abs=0.5)

class TestHRSCStereo1:

    @pytest.mark.parametrize('image, ground',[
          # ((0.5, 0.5, 0), (979590.30174957, 3242179.3919958, 249088.83886381)),
          # ((0.5, 2583.5, 0), (855179.96698611, 3277243.42159, 248427.66616907)),
          # ((12503.5, 0.5, 0), (984596.8817312, 3152386.5706555, 787256.94481508)),
          # ((12503.5, 2583.5, 0), (824462.92414568, 3197824.047306, 787981.15939371)),
          # ((6251.5, 1291.5, 0), (913762.62459356, 3231516.1914323, 503425.73182682))
          ((6251.5, 1291.5, 0), (913776.1812336871, 3231512.357457191, 503425.7355612734)),
          ((0.5, 0.5, 0), (979602.4581277388, 3242175.7082415046, 249088.97789060202)),
          ((0.5, 2583.5, 0), (855191.9410074502, 3277240.29787523, 248427.65484663288)),
          ((12503.5, 0.5, 0), (984612.2803597612, 3152382.0160443066, 787255.9359414595)),
          ((12503.5, 2583.5, 0), (824478.2804830034, 3197820.3642705963, 787980.0517643926))
    ])
    def test_image_to_ground(self, hrsc_stereo_1_model, image, ground):
        gx, gy, gz = ground
        x, y, z = hrsc_stereo_1_model.imageToGround(*image)
        assert x == pytest.approx(gx, abs=100)
        assert y == pytest.approx(gy, abs=100)
        assert z == pytest.approx(gz, abs=100)

    @pytest.mark.parametrize('image, ground',[
          # ((0.5, 0.5, 0), (979590.30174957, 3242179.3919958, 249088.83886381)),
          # ((0.5, 2583.5, 0), (855179.96698611, 3277243.42159, 248427.66616907)),
          # ((12503.5, 0.5, 0), (984596.8817312, 3152386.5706555, 787256.94481508)),
          # ((12503.5, 2583.5, 0), (824462.92414568, 3197824.047306, 787981.15939371)),
          # ((6251.5, 1291.5, 0), (913762.62459356, 3231516.1914323, 503425.73182682))
          ((6251.5, 1291.5, 0), (913776.1812336871, 3231512.357457191, 503425.7355612734)),
          ((0.5, 0.5, 0), (979602.4581277388, 3242175.7082415046, 249088.97789060202)),
          ((0.5, 2583.5, 0), (855191.9410074502, 3277240.29787523, 248427.65484663288)),
          ((12503.5, 0.5, 0), (984612.2803597612, 3152382.0160443066, 787255.9359414595)),
          ((12503.5, 2583.5, 0), (824478.2804830034, 3197820.3642705963, 787980.0517643926))
    ])
    def test_ground_to_image(self, hrsc_stereo_1_model, image, ground):
        y, x = hrsc_stereo_1_model.groundToImage(*ground)
        iy, ix, _ = image

        assert x == pytest.approx(ix, abs=0.5)
        assert y == pytest.approx(iy, abs=0.5)

class TestHRSCStereo2:

    @pytest.mark.parametrize('image, ground',[
          # ((0.5, 0.5, 0), (994968.78141471, 3243624.373581, 150910.83677465)),
          # ((0.5, 2583.5, 0), (840154.182443, 3286852.5036334, 156704.92657188)),
          # ((14263.5, 0.5, 0), (1015683.4735728, 3170802.9252193, 665761.34487006)),
          # ((14263.5, 2583.5, 0), (802816.11887483, 3229529.8393814, 674042.87178595)),
          # ((7131.5, 1291.5, 0), (915544.01372502, 3243112.89855, 419540.13516932))
          ((7131.5, 1291.5, 0), (915561.6002628063, 3243108.0221202467, 419539.4600493548)),
          ((0.5, 0.5, 0), (994983.8770025282, 3243619.7787126494, 150910.07885617757)),
          ((0.5, 2583.5, 0), (840169.1316321647, 3286848.7002639165, 156704.55683200277)),
          ((14263.5, 0.5, 0), (1015704.1721659054, 3170796.569969528, 665760.0501533676)),
          ((14263.5, 2583.5, 0), (802836.7715344077, 3229524.77329203, 674042.5500676908))
    ])
    def test_image_to_ground(self, hrsc_stereo_2_model, image, ground):
        gx, gy, gz = ground
        x, y, z = hrsc_stereo_2_model.imageToGround(*image)
        assert x == pytest.approx(gx, abs=100)
        assert y == pytest.approx(gy, abs=100)
        assert z == pytest.approx(gz, abs=100)

    @pytest.mark.parametrize('image, ground',[
          # ((0.5, 0.5, 0), (994968.78141471, 3243624.373581, 150910.83677465)),
          # ((0.5, 2583.5, 0), (840154.182443, 3286852.5036334, 156704.92657188)),
          # ((14263.5, 0.5, 0), (1015683.4735728, 3170802.9252193, 665761.34487006)),
          # ((14263.5, 2583.5, 0), (802816.11887483, 3229529.8393814, 674042.87178595)),
          # ((7131.5, 1291.5, 0), (915544.01372502, 3243112.89855, 419540.13516932))
          ((7131.5, 1291.5, 0), (915561.6002628063, 3243108.0221202467, 419539.4600493548)),
          ((0.5, 0.5, 0), (994983.8770025282, 3243619.7787126494, 150910.07885617757)),
          ((0.5, 2583.5, 0), (840169.1316321647, 3286848.7002639165, 156704.55683200277)),
          ((14263.5, 0.5, 0), (1015704.1721659054, 3170796.569969528, 665760.0501533676)),
          ((14263.5, 2583.5, 0), (802836.7715344077, 3229524.77329203, 674042.5500676908))
    ])
    def test_ground_to_image(self, hrsc_stereo_2_model, image, ground):
        y, x = hrsc_stereo_2_model.groundToImage(*ground)
        iy, ix, _ = image

        assert x == pytest.approx(ix, abs=0.5)
        assert y == pytest.approx(iy, abs=0.5)
