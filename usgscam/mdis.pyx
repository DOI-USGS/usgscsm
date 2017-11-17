import numpy as np
import ast
import json
from cython.operator cimport dereference as deref

from cycsm.csm import EcefCoord, ImageCoord, Set, Type
from cycsm.isd cimport Isd
from cycsm.version import Version
#from cycsm.correlationmodel cimport NoCorrelationModel, CppNoCorrelationModel

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

    def imageToGround(self, line, sample, double height, double precision=0.001):
        """
        Given an image line/sample and a height, compute the Ellipse Centered
        Ellipse Fixed coordinate intersection.

        In the case of MDIS NAC and WAC this is computed using the collinearity
        equations.

        Parameters
        ----------
        line : numeric
               The line number in the image
        sample : numeric
                 The sample number in the image

        height : numeric
                 the height above the sphere/ellipsoid at which to compute
                 the look vector intersection

        Returns
        -------
         : list
           The x, y, z ground intersection coordinate in ECEF
        """
        pt = ImageCoord(line, sample)
        e = self.thisptr.imageToGround(pt._ptr, height, precision)
        return [e.x, e.y, e.z]

    def groundToImage(self, x, y, z, double precision=0.001):
        """
        Given an Earth Centered Earth Fixed (ECEF) ground coordinate,
        compute the look intersection into the image.

        This utilizes the collinearity equations.

        Parameters
        ----------
        x : numeric
            The 'ground' x coordinate

        y : numeric
            The 'ground' y coordinate

        z : numeric
            The 'ground' z coordinate

        Returns
        -------
         : list
           of line/sample coordinates in the image
        """
        ground = EcefCoord(x,y,z)
        pt = ImageCoord()
        e = self.thisptr.groundToImage(ground._ptr, precision)
        return [e.line, e.samp]

    def image_to_proximate_imaging_locus(self, line, sample, x, y, z, double precision=0.001):
        """
        Compute a proximate imaging locus, a vector approximation of the imaging locus for the
        given line/sample nearest the given ECEF x, y, z.  Precision refers to the point (location)
        but not the orientation (direction).

        Parameters
        ----------
        line : numeric
               The line number in the image

        sample : numeric
                 The sample number in the image

        x : numeric
            The 'ground' x coordinate

        y : numeric
            The 'ground' y coordinate

        z : numeric
            The 'ground' z coordinate

        Returns
        -------
        res.point : dict
                    containing 'x', 'y', 'z' coordinates
        res.direction : dict
                        containing 'x', 'y', 'z' vector orientations
        """
        i = ImageCoord(line, sample)
        g = EcefCoord(x, y, z)
        res = self.thisptr.imageToProximateImagingLocus(i._ptr, g._ptr, precision)
        return res.point, res.direction

    def image_to_remote_imaging_locus(self, line, sample, double precision=0.001):
        """
        Compute a proximate imaging locus, a vector approximation of the imaging locus for the
        given line/sample nearest the given ECEF x, y, z.  Precision refers to the point (location)
        but not the orientation (direction).

        Parameters
        ----------
        line : numeric
               The line number in the image

        sample : numeric
                 The sample number in the image

        Returns
        -------
        res.point : dict
                    containing 'x', 'y', 'z' coordinates
        res.direction : dict
                        containing 'x', 'y', 'z' vector orientations
        """
        i = ImageCoord(line, sample)
        res = self.thisptr.imageToRemoteImagingLocus(i._ptr, precision)
        return res.point, res.direction

    def compute_sensor_partials(self, index, x, y, z, line=None, sample=None, double precision=0.001):
        g = EcefCoord(x, y, z)
        if line != None and sample != None:
            i = ImageCoord(line, sample)
            partials = self.thisptr.computeSensorPartials(<int>index, <CppImageCoord>i._ptr, <CppEcefCoord>g._ptr, <double>precision)
        else:
            partials = self.thisptr.computeSensorPartials(<int>index, <CppEcefCoord>g._ptr, <double>precision)
        return partials

    def compute_all_sensor_partials(self, x, y, z, param=0, line=None, sample=None, double precision=0.001):
        g = EcefCoord(x,y,z)
        pset = Set(param)
        if line != None and sample != None:
            i = ImageCoord(line, sample)
            partials = self.thisptr.computeAllSensorPartials(<CppImageCoord>i._ptr, <CppEcefCoord>g._ptr, <CppSet>pset, <double>precision)
        else:
            partials = self.thisptr.computeAllSensorPartials(<CppEcefCoord>g._ptr, pset, <double>precision)
        return partials

    def compute_ground_partials(self, x, y, z):
        g = EcefCoord(x,y,z)
        return self.thisptr.computeGroundPartials(g._ptr)

    def get_parameter_type(self, index):
        return Type(self.thisptr.getParameterType(index))

    def set_parameter_type(self, int index, ptype):
        ptype = Type(ptype)
        self.thisptr.setParameterType(index, ptype);

    @property
    def imagesize(self):
        """
        The total number of lines/samples in the image.

        Returns:
         : list
            lines / sample count
        """
        res = self.thisptr.getImageSize()
        return res.line, res.samp

    @property
    def imagestart(self):
        """
        The line and sample start.

        Returns
        -------
         : list
           line/sample start locations
        """
        res = self.thisptr.getImageStart()
        return res.line, res.samp

    @property
    def state(self):
        """
        The current state of the model, as a string, that can be used to
        reinstantiate the model using the replace_model_state method.
        """
        return self.thisptr.getModelState().decode()

    @property
    def name(self):
        """
        Name of the model.
        """
        return self.thisptr.getModelName().decode()

    @property
    def sensortype(self):
        return self.thisptr.getSensorType().decode()

    @property
    def sensormode(self):
        return self.thisptr.getSensorMode().decode()

    @property
    def imagerange(self):
        """
        Get the start and stop pixel counts as a tuple in
        the form: ((min_pt.line, min_pt.sample), (max_pt.line, max_pt.sample))
        """
        min_pt, max_pt = self.thisptr.getValidImageRange()
        return (min_pt['line'], min_pt['samp']),\
               (max_pt['line'], max_pt['samp'])

    @property
    def heightrange(self):
        return self.thisptr.getValidHeightRange()

    #@property
    #def version(self):
    #    return self.thisptr.getVersion().major#.version

    def sensor_time_position(self, time):
        """
        Compute the sensor position given an image time.

        Parameters
        ----------
        time : numeric
               The image time

        Returns
        -------
         : list
           The sensor position vector in ECEF coordinates (m).
        """
        res = self.thisptr.getSensorPosition(<double> time)
        return [res.x, res.y, res.z]

    def sensor_coordinate_position(self, line, sample):
        """
        Compute the sensor position given a line/sample in the image.

        Parameters
        ----------
        line : numeric
               The line number in the image
        sample : numeric
                 The sample number in the image

        Returns
        -------
         : list
           The sensor position vector in ECEF coordinates (m).
        """
        i = ImageCoord(line, sample)
        res = self.thisptr.getSensorPosition(<CppImageCoord> i._ptr)
        return [res.x, res.y, res.z]

    def sensor_time_velocity(self, time):
        res = self.thisptr.getSensorVelocity(<double> time)
        return [res.x, res.y, res.z]

    def sensor_coordinate_velocity(self, line, sample):
        i = ImageCoord(line, sample)
        res = self.thisptr.getSensorVelocity(<CppImageCoord> i._ptr)
        return [res.x, res.y, res.z]

    def illumination_direction(self, ground_point):
        """
        Given an ellipse centered ellipse fixed coordinate, compute the
        illumination direction.

        Parameters
        ----------
        ground_point : list
                       of x, y, z ECEF coordiantes

        Returns
        -------
         : list
           x, y, z, vector describing the illumination direction
        """
        pt = EcefCoord(*ground_point)
        res = self.thisptr.getIlluminationDirection(pt._ptr)
        return [res.x, res.y, res.z]

cdef class MdisPlugin:
    cdef:
        CppMdisPlugin *thisptr

    def __cinit__(self):
        self.thisptr = new CppMdisPlugin()

    def __dealloc__(self):
        del self.thisptr

    @property
    def name(self):
        return self.thisptr.getPluginName().decode()

    @property
    def manufacturer(self):
        return self.thisptr.getManufacturer().decode()

    @property
    def releasedate(self):
        return self.thisptr.getReleaseDate().decode()

    #@property
    #def csmversion(self):
    #    return self.thisptr.getCsmVersion().major#.version

    @property
    def nmodels(self):
        return self.thisptr.getNumModels()

    
    def modelfamily(self, modelindex):
        return self.thisptr.getModelFamily(modelindex).decode()

    def modelname_from_state(self, state):
        state = state.encode()
        res = self.thisptr.getModelNameFromModelState(state).decode()
        return res

    def modelname(self, modelindex):
        return self.thisptr.getModelName(modelindex).decode()

    def can_model_be_constructed_from_state(self, name, state):
        name = name.encode()
        state = state.encode()
        return self.thisptr.canModelBeConstructedFromState(name, state)

    def can_isd_be_converted_to_model_state(self, Isd isd, modelname):
        print(modelname, type(isd))
        modelname = modelname.encode()
        return self.thisptr.canISDBeConvertedToModelState(isd.thisptr[0], modelname)

    def convert_isd_to_state(self, Isd isd, modelname):
        modelname = modelname.encode()
        return json.loads(self.thisptr.convertISDToModelState(isd.thisptr[0], modelname).decode())

    def check_isd_construction(self, Isd isd, modelname):
        modelname = str(modelname).encode()
        return self.thisptr.canModelBeConstructedFromISD(isd.thisptr[0], modelname)

    def from_isd(self, Isd isd, modelname):
        modelname = str(modelname).encode()
        return MdisNacSensorModel.factory(dynamic_cast_model_ptr(self.thisptr.constructModelFromISD(isd.thisptr[0], modelname)))

    def from_state(self, state):
        state = state.encode()
        #self.thisptr.constructModelFromState(state)
        return MdisNacSensorModel.factory(dynamic_cast_model_ptr(self.thisptr.constructModelFromState(state)))
