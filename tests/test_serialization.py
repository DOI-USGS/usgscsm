import pickle

import pytest

import usgscam as cam

# TODO: This should cycle through all of our JSON instantiation examples
def test_pickle_io(generic_model):
    pfile = pickle.dumps(generic_model, 2)
    loaded_model = pickle.loads(pfile)
    assert generic_model.name == loaded_model.name
    assert isinstance(loaded_model, cam.genericframe.SensorModel)
