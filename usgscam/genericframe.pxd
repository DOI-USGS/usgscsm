from libcpp.string cimport string
from libcpp.cast cimport dynamic_cast
from libcpp cimport bool
from libcpp.pair cimport pair
from libcpp.vector cimport vector

from cycsm.isd cimport CppIsd
from cycsm.csm cimport CppEcefCoord, CppImageCoord, CppImageVector, CppEcefVector, CppEcefLocus, CppSet, CppType
from cycsm.model cimport CppModel
from cycsm.version cimport CppVersion
from cycsm.correlationmodel cimport CppNoCorrelationModel
from cycsm.rastergm cimport CppSensorPartials

cdef extern from "UsgsAstroFrameSensorModel.h":
    cdef cppclass CppFrameSensorModel "UsgsAstroFrameSensorModel":
        CppUsgsAstroFrameNacSensorModel() except +
        CppEcefCoord imageToGround(CppImageCoord imagePt, double height, double precision) except +
        CppImageCoord groundToImage(CppEcefCoord groundPt, double desiredPrecision) except +
        CppImageCoord getImageStart()
        CppImageVector getImageSize()

        string getModelState()
        CppEcefVector getIlluminationDirection(CppEcefCoord &groundPoint)
        CppEcefCoord getSensorPosition(CppImageCoord &imagePt)
        CppEcefVector getSensorVelocity(CppImageCoord &imagePt)
        CppEcefLocus imageToProximateImagingLocus(CppImageCoord &imagePt,
                                                   CppEcefCoord &groundPt,
                                                   double desiredPrecision)
        CppEcefLocus imageToRemoteImagingLocus(CppImageCoord &imagePt,
                                                  double desiredPrecision)
        CppSensorPartials computeSensorPartials(int index,
                                                CppImageCoord &imagePt,
                                                CppEcefCoord &groundPt,
                                                double desiredPrecision) except +

        # Newly implemented
        CppSensorPartials computeSensorPartials(int index,
                                                CppEcefCoord &groundPt,
                                                double desiredPrecision) except +


        vector[CppSensorPartials] computeAllSensorPartials(CppEcefCoord &groundPt,
                                                           CppSet pset,
                                                           double desiredPrecision) except +

        vector[CppSensorPartials] computeAllSensorPartials(CppImageCoord &imagePt,
                                                           CppEcefCoord &groundPt,
                                                           CppSet pset,
                                                           double desiredPrecision) except +

        CppType getParameterType(int index) except +
        void setParameterType(int index, CppType pType) except +

        vector[double] computeGroundPartials(CppEcefCoord &groundPt)

        pair[CppImageCoord, CppImageCoord] getValidImageRange()
        pair[double, double] getValidHeightRange()
        CppEcefVector getSensorVelocity(double time) except +
        CppEcefCoord getSensorPosition(double time) except +
        CppVersion getVersion()
        string getModelName()
        string getSensorType()
        string getSensorMode()
        #CppNoCorrelationModel getCorrelationModel() # Not wrapped


cdef extern from "UsgsAstroFramePlugin.h":
    cdef cppclass CppUsgsAstroFramePlugin "UsgsAstroFramePlugin":
        CppUsgsAstroFramePlugin() except +

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
        bool canISDBeConvertedToModelState(CppIsd isd, const string modelName) except *
        string convertISDToModelState(CppIsd isd, const string modelname) except +
        CppModel *constructModelFromISD(CppIsd &isd, string &modelname) except +
        CppModel *constructModelFromState(string modelState) except +

#  For casting from the CSM Model into our specific camera model - I hope.
cdef extern from *:
    CppFrameSensorModel* dynamic_cast_model_ptr "dynamic_cast<UsgsAstroFrameSensorModel*>"(CppModel*) except NULL
