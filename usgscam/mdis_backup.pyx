import numpy as np
import ast
from cython.operator cimport dereference as deref

from libcpp.memory cimport unique_ptr, shared_ptr
from libcpp.string cimport string
from libcpp.cast cimport dynamic_cast
from libcpp cimport bool

#ISD
cdef extern from "Isd.h" namespace "csm":
    cdef cppclass CppIsd "csm::Isd":
        Isd() except +
        void addParam(string key, string value)
        string param(string key, int instance)
        void clearParams(string&)
        void clearAllParams()

cdef class Isd:
    cdef CppIsd *thisptr

    def __init__(self):
        self.thisptr = new CppIsd()

    def __dealloc__(self):
        del self.thisptr

    def addparam(self, key, value):
        key = str(key).encode()
        if isinstance(value, np.ndarray):
            value = value.tolist()
        value = str(value).encode()
        self.thisptr.addParam(key, value)

    def param(self, key, instance=0):
        key = str(key).encode()
        value = self.thisptr.param(key, instance)
        try:
            return ast.literal_eval(value.decode())
        except:
            return value.decode()

    def clear_params(self, key):
        key = str(key).encode()
        self.thisptr.clearParams(key)

    def clear_all_params(self):
        self.thisptr.clearAllParams()



# Image Coord and Ecef Coords
cdef extern from "csm.h" namespace "csm::param":
    cdef cppclass CppImageCoord "csm::ImageCoord":
        ImageCoord() except +
        double line
        double samp

    cdef cppclass CppEcefCoord "csm::EcefCoord":
        EcefCoord() except +
        double x
        double y
        double z

cdef class ImageCoord:
    cdef CppImageCoord *thisptr

    def __init__(self, double line=0.0, double sample=0.0):
        self.thisptr = new CppImageCoord()
        self.thisptr.line = line
        self.thisptr.samp = sample

    def __dealloc__(self):
        del self.thisptr

    def __repr__(self):
        return 'Line: {}, Sample: {}'.format(self.thisptr.line,
                                             self.thisptr.samp)

    @property
    def sample(self):
        return self.thisptr.samp

    @sample.setter
    def sample(self, double value):
        self.thisptr.samp = value

    @property
    def line(self):
        return self.thisptr.line

    @line.setter
    def line(self, double value):
        self.thisptr.line = value

cdef class EcefCoord:
    cdef CppEcefCoord *thisptr

    def __init__(self, double x=0.0, double y=0.0, double z=0.0, _raw=False):
        if _raw is False:
            self.thisptr = new CppEcefCoord()
            self.x = x
            self.y = y
            self.z = z

    def __dealloc__(self):
        if self.thisptr is not NULL:
            del self.thisptr
        self.thisptr = NULL

    def __repr__(self):
        return "{}, {}, {}".format(self.x, self.y, self.z)

    @staticmethod
    cdef factory(CppEcefCoord obj):
        py_obj = EcefCoord.__new__(EcefCoord, _raw=True)
        (<EcefCoord>py_obj).thisptr = new CppEcefCoord()
        py_obj.x = obj.x
        py_obj.y = obj.y
        py_obj.z = obj.z
        return py_obj

    @property
    def x(self):
        return self.thisptr.x
    @x.setter

    def x(self, double value):
        self.thisptr.x = value
    @property
    def y(self):
        return self.thisptr.y
    @y.setter
    def y(self, double value):
        self.thisptr.y = value

    @property
    def z(self):
        return self.thisptr.z
    @z.setter
    def z(self, double value):
        self.thisptr.z = value



# Model
cdef extern from "Model.h" namespace "csm":
    cdef cppclass CppModel "csm::Model":
        Model() except +

"""cdef class Model:
    cdef:
        CppModel *thisptr

    def __init__(self):
        self.thisptr = new CppModel()"""

cdef extern from "MdisNacSensorModel.h":
    cdef cppclass CppMdisNacSensorModel "MdisNacSensorModel":
        MdisNacSensorModel() except +
        CppEcefCoord imageToGround(CppImageCoord imagePt, double height, double precision)
        string getModelState()
        string getModelName()

cdef class MdisNacSensorModel:
    cdef:
        CppMdisNacSensorModel *thisptr

    def __cinit__(self, _raw=False):
        if _raw is False:
            self.thisptr = new CppMdisNacSensorModel()
        else:
            self.thisptr = NULL

    def __dealloc__(self):
        del self.thisptr

    @staticmethod
    cdef factory(CppMdisNacSensorModel *obj):
        py_obj = result = MdisNacSensorModel.__new__(MdisNacSensorModel, _raw=True)
        (<MdisNacSensorModel>py_obj).thisptr = obj
        return result

    def imageToGround(self, ImageCoord pt, double height, double precision=0.001):
        #return self.thisptr.imageToGround(pt.thisptr[0], height, precision)
        return EcefCoord.factory(self.thisptr.imageToGround(pt.thisptr[0], height, precision))

    @property
    def state(self):
        """
        The current state of the model, as a string, that can be used to
        reinstantiate the model using the replace_model_state method.
        """
        return self.thisptr.getModelState().decode()

    @property
    def name(self):
        return self.thisptr.getModelName().decode()



cdef extern from "MdisPlugin.h":
    cdef cppclass CppMdisPlugin "MdisPlugin":
        MdisPlugin() except +

        string getPluginName()
        string getManufacturer()
        #string getReleaseDate()
        #string getCsmVersion
        size_t getNumModels()
        string getModelName(size_t modelIndex)
        bool canModelBeConstructedFromISD(CppIsd isd, const string modelname)
        string convertISDToModelState(CppIsd isd, const string modelname)
        CppModel *constructModelFromISD(CppIsd &isd, string &modelname)

#  For casting from the CSM Model into our specific camera model - I hope.
cdef extern from *:
    CppMdisNacSensorModel* dynamic_cast_model_ptr "dynamic_cast<MdisNacSensorModel*>"(CppModel*) except NULL

cdef class MdisPlugin:
    cdef:
        CppMdisPlugin *thisptr

    def __init__(self):
        self.thisptr = new CppMdisPlugin()

    def __dealloc__(self):
        del self.thisptr

    @property
    def name(self):
        return self.thisptr.getPluginName().decode()

    @property
    def manufacturer(self):
        return self.thisptr.getManufacturer().decode()

    #@property
    #def releasedate(self):
    #    return self.thisptr.getReleaseDate()

    #@property
    #def version(self):
    #    return self.thisptr.getCsmVersion()

    @property
    def nmodels(self):
        return self.thisptr.getNumModels()

    def modelname(self, modelindex):
        return self.thisptr.getModelName(modelindex).decode()

    def check_isd_construction(self, Isd isd, modelname):
        modelname = str(modelname).encode()
        return self.thisptr.canModelBeConstructedFromISD(isd.thisptr[0], modelname)

    def from_isd(self, Isd isd, modelname):
        modelname = str(modelname).encode()
        return MdisNacSensorModel.factory(dynamic_cast_model_ptr(self.thisptr.constructModelFromISD(isd.thisptr[0], modelname)))

        #dc = dynamic_cast(MdisNacSensorModel.thisptr, state.thisptr[0])
        #return dc
