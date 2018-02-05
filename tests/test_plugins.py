import json
import os

import pytest

import usgscam as cam
from cycsm import isd
import cycsm.csm as csm

data_path = os.path.dirname(__file__)


@pytest.fixture(scope="session")
def mdis_plugin():
    return cam.mdis.MdisPlugin()

@pytest.fixture(scope="session")
def genericls_plugin():
    return cam.genericls.GenericLsPlugin()

@pytest.fixture
def mdis_isd(scope="session"):
    isd_file = os.path.join(data_path, 'EN1007907102M.json')
    with open(isd_file, 'r') as f:
        return isd.Isd.load(f)

@pytest.fixture
def genericls_isd(scope="session"):
    isd_file = os.path.join(data_path, 'J03_046060_1986_XN_18N282W_8bit_keywords.lis')
    return isd.Isd.read_socet_file(isd_file)

@pytest.fixture
def nac_state(scope="session"):
    with open(os.path.join(data_path,'nac_state.json'), 'r') as f:
        return json.load(f)

@pytest.fixture
def ctx_state(scope="session"):
    path = os.path.join(data_path, 'J03_046060_1986_XN_18N282W_8bit_state.json')
    with open(path, 'r') as f:
        return json.load(f)

@pytest.mark.parametrize('plugin, expected', [(mdis_plugin(), 1),
                                              (genericls_plugin(), 1)])
def test_nmodels(plugin, expected):
    assert plugin.nmodels == expected


@pytest.mark.parametrize('plugin, expected', [(mdis_plugin(), "20170425"),
                                              (genericls_plugin(), "20171230")])
def test_releasedate(plugin, expected):
    assert plugin.releasedate == expected


@pytest.mark.parametrize('plugin, expected',
                         [(mdis_plugin(), "UsgsAstroFrameMdisPluginCSM"),
                          (genericls_plugin(), "USGS_ASTRO_LINE_SCANNER_PLUGIN")])
def test_plugin_name(plugin, expected):
    assert plugin.name == expected


@pytest.mark.parametrize('plugin, expected',
                         [(mdis_plugin(), "USGS_ASTRO_FRAME_SENSOR_MODEL"),
                          (genericls_plugin(), "USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL")])
def test_modelname(plugin, expected):
    assert plugin.modelname(1) == expected


@pytest.mark.parametrize('plugin, expected', [(mdis_plugin(), "Raster"),
                                              (genericls_plugin(), "Raster")])
def test_modelfamily(plugin, expected):
    assert plugin.modelfamily(1) == expected


@pytest.mark.parametrize('plugin, state',
                         [(mdis_plugin(), {'model_name':'USGS_ASTRO_FRAME_SENSOR_MODEL'}),
                          (genericls_plugin(), {"STA_SENSOR_MODEL_NAME":"USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL"})])
def test_modelname_from_state(plugin, state):
    name = plugin.modelname_from_state(json.dumps(state))
    assert name == state[list(state)[0]]


@pytest.mark.parametrize('plugin, state',
                         [(mdis_plugin(), {'bad_name':'USGS_ASTRO_FRAME_SENSOR_MODEL'}),
                          (genericls_plugin(), {"BADKEY_GOODVALUE":"USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL"})])
def test_modelname_from_state_bad_key(plugin, state):
    with pytest.raises(RuntimeError) as err:
        name = plugin.modelname_from_state(json.dumps(state))
    assert "key in the model" in str(err.value)


@pytest.mark.parametrize('plugin, state',
                         [(mdis_plugin(), {'model_name':'foo'}),
                          (genericls_plugin(), {"STA_SENSOR_MODEL_NAME":"foo"})])
def test_modelname_from_state_bad_name(plugin, state):
    with pytest.raises(RuntimeError) as err:
        name = plugin.modelname_from_state(json.dumps(state))
    assert "Sensor model not supported." in str(err.value)


@pytest.mark.parametrize('plugin, name, state',
                         [(mdis_plugin(), "USGS_ASTRO_FRAME_SENSOR_MODEL", nac_state()),
                          (genericls_plugin(), "USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL", ctx_state())])
def test_can_model_be_constructed_from_state(plugin, name, state):
    try:
        state = json.dumps(state)
    except: pass
    constructible = plugin.can_model_be_constructed_from_state(name, state)
    assert constructible == True

    name = 'foo'
    constructible = plugin.can_model_be_constructed_from_state(name, state)
    assert constructible == False


@pytest.mark.parametrize('plugin, name, i',
                         [(mdis_plugin(), 'USGS_ASTRO_FRAME_SENSOR_MODEL', mdis_isd()),
                          (genericls_plugin(), "USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL", genericls_isd())])
def test_can_isd_be_converted_to_model_state(plugin, name, i):
    res = plugin.can_isd_be_converted_to_model_state(i, name)
    assert res == True


@pytest.mark.parametrize('plugin, name, i, state',
                         [(mdis_plugin(), 'USGS_ASTRO_FRAME_SENSOR_MODEL', mdis_isd(), nac_state()),
                          (genericls_plugin(), "USGS_ASTRO_LINE_SCANNER_PLUGIN", genericls_isd(), ctx_state())])
def test_convert_isd_to_model_state(plugin, name, i, state):
    pstate = plugin.convert_isd_to_state(i, name)
    for k, v in pstate.items():
        v2 = state.get(k, None)
        assert v == v2

@pytest.mark.parametrize('plugin, i, name, param',
                         [(mdis_plugin(), mdis_isd(),
                           'USGS_ASTRO_FRAME_SENSOR_MODEL', "focal_length"),
                          (genericls_plugin(), genericls_isd(),
                           "USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL", "FOCAL")])
def test_convert_isd_to_model_state_bad(plugin, i, name, param):
    # Trash the isd and check error handling
    i.clear_params(param)
    with pytest.raises(RuntimeError) as err:
        state = plugin.convert_isd_to_state(i, name)
    assert "Sensor model support data" in str(err.value)


@pytest.mark.parametrize('plugin, state, instance', [(mdis_plugin(), nac_state(), cam.mdis.FrameSensorModel),
                                              (genericls_plugin(), ctx_state(), cam.genericls.GenericLsSensorModel)])
def test_construct_model_from_state(plugin, state, instance):
    camera = plugin.from_state(json.dumps(state))
    assert isinstance(camera, instance)

@pytest.mark.parametrize('plugin, state, instance', [(mdis_plugin(), nac_state(), cam.mdis.FrameSensorModel),
                                              (genericls_plugin(), ctx_state(), cam.genericls.GenericLsSensorModel)])
def test_construct_model_from_bad_state(plugin, state, instance):
    # Remove the an entry from the state, making it invalid
    k = list(state.keys())[0]
    state.pop(k, None)
    with pytest.raises(RuntimeError) as err:
        camera = plugin.from_state(json.dumps(state))
    assert "Model state is not" in str(err.value)

@pytest.mark.parametrize('plugin, isd, modelname',
                         [(mdis_plugin(), mdis_isd(), 'USGS_ASTRO_FRAME_SENSOR_MODEL'),
                          (genericls_plugin(), genericls_isd(), "USGS_ASTRO_LINE_SCANNER_SENSOR_MODEL")])
def test_can_mode_be_constructed_from_isd(plugin, isd, modelname):
    assert plugin.check_isd_construction(isd, modelname)
