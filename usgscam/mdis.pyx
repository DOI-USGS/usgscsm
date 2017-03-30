import numpy as np
import ast
from cython.operator cimport dereference as deref

from cycsm.csm import EcefCoord, ImageCoord
from cycsm.isd cimport Isd

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
        #TODO: Return the straight floats without the update to an EcefCoord
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
    def imagerange(self):
        raise NotImplementedError
        #self.thisptr.getValidImageRange()

    @property
    def heightrange(self):
        raise NotImplementedError
        #return self.thisptr.getValidHeightRange()

    def sensor_time_position(self, time):
        raise NotImplementedError
        #res = self.thisptr.getSensorPosition(<double> time)
        #return [res.x, res.y, res.z]

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
        raise NotImplementedError

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
