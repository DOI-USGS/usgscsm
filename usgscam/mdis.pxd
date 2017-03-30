from libcpp.string cimport string
from libcpp.cast cimport dynamic_cast
from libcpp cimport bool
from libcpp.pair cimport pair

from cycsm.isd cimport CppIsd
from cycsm.csm cimport CppEcefCoord, CppImageCoord, CppImageVector, CppEcefVector, CppEcefLocus
from cycsm.model cimport CppModel

cdef extern from "MdisNacSensorModel.h":
    cdef cppclass CppMdisNacSensorModel "MdisNacSensorModel":
        CppMdisNacSensorModel() except +
        CppEcefCoord imageToGround(CppImageCoord imagePt, double height, double precision)
        CppImageCoord groundToImage(CppEcefCoord groundPt, double desiredPrecision)
        CppImageCoord getImageStart()
        CppImageVector getImageSize()

        string getModelState()
        string getModelName()
        CppEcefVector getIlluminationDirection(CppEcefCoord &groundPoint)
        CppEcefCoord getSensorPosition(CppImageCoord &imagePt)
        CppEcefCoord getSensorPosition(double time)
        CppEcefVector getSensorVelocity(CppImageCoord &imagePt)
        CppEcefVector getSensorVelocity(double time)
        CppEcefLocus imageToProximateImagingLocus(CppImageCoord &imagePt,
                                                   CppEcefCoord &groundPt,
                                                   double desiredPrecision)
        CppEcefLocus imageToRemoteImagingLocus(CppImageCoord &imagePt,
                                                  double desiredPrecision)

        #TODO: IMPLEMENT FOR CYTHON
        # CppSensorPartials computeSensorPartials(int index, CppEcefCoord &groundPt)
        # CppVersion getVersion()

        #TODO: IMPLEMENT FOR CYTHON AND CPP
        pair[CppImageCoord, CppImageCoord] getValidImageRange()
        pair[double, double] getValidHeightRange()

cdef extern from "MdisPlugin.h":
    cdef cppclass CppMdisPlugin "MdisPlugin":
        CppMdisPlugin() except +

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
