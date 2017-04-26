from libcpp.string cimport string
from libcpp.cast cimport dynamic_cast
from libcpp cimport bool
from libcpp.pair cimport pair

from cycsm.isd cimport CppIsd
from cycsm.csm cimport CppEcefCoord, CppImageCoord, CppImageVector, CppEcefVector, CppEcefLocus
from cycsm.model cimport CppModel
from cycsm.version cimport CppVersion

cdef extern from "MdisNacSensorModel.h":
    cdef cppclass CppMdisNacSensorModel "MdisNacSensorModel":
        CppMdisNacSensorModel() except +
        CppEcefCoord imageToGround(CppImageCoord imagePt, double height, double precision) except +
        CppImageCoord groundToImage(CppEcefCoord groundPt, double desiredPrecision) except +
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
        string getModelNameFromModelState(const string modelState) except +
        string getReleaseDate()
        CppVersion getCsmVersion()
        size_t getNumModels()
        string getModelName(size_t modelIndex)
        string getModelFamily(size_t modelIndex)
        bool canModelBeConstructedFromISD(CppIsd isd, const string modelname) except +
        bool canModelBeConstructedFromState(const string modelName, const string modelState) except +
        bool canISDBeConvertedToModelState(CppIsd isd, const string modelName) except +
        string convertISDToModelState(CppIsd isd, const string modelname) except +
        CppModel *constructModelFromISD(CppIsd &isd, string &modelname) except +
        CppModel *constructModelFromState(string modelState) except +

#  For casting from the CSM Model into our specific camera model - I hope.
cdef extern from *:
    CppMdisNacSensorModel* dynamic_cast_model_ptr "dynamic_cast<MdisNacSensorModel*>"(CppModel*) except NULL
