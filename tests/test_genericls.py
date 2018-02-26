import os
import json
import pytest

from cycsm import isd
import cycsm.csm as csm
import usgscam as cam

data_path = os.path.dirname(__file__)

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

class TestHRSC:

    @pytest.mark.parametrize('image, ground',[
                              ((1291.5, 14351.5, 0), (1050372.2680538, 995148.04296401, -3054455.4812471))
    ])
    def test_image_to_ground(self, hrsc_model, image, ground):
        gx, gy, gz = ground
        x, y, z = hrsc_model.imageToGround(*image)
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
