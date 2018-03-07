import pickle

import pytest

import usgscam as cam

def test_pickle_io(generic_model):
    pfile = pickle.dumps(generic_model, 2)
    loaded_model = pickle.loads(pfile)
    assert generic_model.name == loaded_model.name
    assert isinstance(loaded_model, cam.genericframe.SensorModel)

def test_ls_pickle_io(ctx_model):
    pfile = pickle.dumps(ctx_model, 2)
    loaded_model = pickle.loads(pfile)
    assert ctx_model.name == loaded_model.name
    assert isinstance(loaded_model, cam.genericls.SensorModel)
