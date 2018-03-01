import json
import os

import pytest

from cycsm import isd
import cycsm.csm as csm
import usgscam as cam

data_path = os.path.dirname(__file__)


#TODO: This should be a single fixture that accepts the json file as an arg

@pytest.fixture(params=['CW1071364100B_IU_5.json', 'cassini_nac.json', 'EN1007907102M.json'])
def generic_model(request):
    csm_isd = isd.Isd()
    with open(os.path.join(data_path,request.param), 'r') as f:
        d = json.load(f)
    for k, v in d.items():
        csm_isd.addparam(k, v)

    plugin = cam.genericframe.Plugin()
    return plugin.from_isd(csm_isd, plugin.modelname(1))

@pytest.fixture
def mdis_wac_model():
    csm_isd = isd.Isd()
    with open(os.path.join(data_path,'CW1071364100B_IU_5.json'), 'r') as f:
        d = json.load(f)
    for k, v in d.items():
        csm_isd.addparam(k, v)

    plugin = cam.genericframe.Plugin()
    return plugin.from_isd(csm_isd, plugin.modelname(1))

@pytest.fixture
def ctx_model():
    path = os.path.join(data_path, 'J03_046060_1986_XN_18N282W_8bit_keywords.lis')
    csm_isd = isd.Isd.read_socet_file(path)
    plugin = cam.genericls.Plugin()
    return plugin.from_isd(csm_isd, plugin.modelname(1))

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

@pytest.fixture
def cassini_model():
    path = os.path.join(data_path,'cassini_nac.json')
    with open(path, 'r') as f:
        csm_isd = isd.Isd.load(f)
    plugin = cam.genericframe.Plugin()
    return plugin.from_isd(csm_isd, plugin.modelname(1))
