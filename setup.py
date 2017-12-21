import os
import pkg_resources
import sys
import sysconfig
from setuptools import setup, Extension, find_packages
from Cython.Build import cythonize

# Look for the csmapi headers in the standard location
incdir = os.path.dirname(sysconfig.get_path('include'))

INCLUDE_DIRS = ['include/json', 'include/mdis', 'include/orex',
                'include/genericls',
                'include/', incdir, os.path.join(incdir, 'csm')]
LIBRARY_DIRS = []  # This assumes that libcsmapi is installed in a standard place
LIBRARIES = ['csmapi']
COMPILE_ARGS = ['-g', '-std=c++11']#, '-stdlib=libc++']

if sys.platform == 'darwin':
    COMPILE_ARGS.append('-mmacosx-version-min=10.9')
elif sys.platform.startswith("win"):
    try:
        INCLUDE_DIRS.append(os.path.join(os.environ['LIBRARY_INC'], 'csm'))
    except: pass
    COMPILE_ARGS = []

def generate_extension(path_name, sources):
    return Extension(path_name,
                sources=sources,
                extra_compile_args=COMPILE_ARGS,
                language='c++',
                include_dirs=INCLUDE_DIRS,
                runtime_library_dirs=LIBRARY_DIRS,
                libraries=LIBRARIES)

# Create the extensions
extensions = [generate_extension('usgscam.mdis', ['usgscam/mdis.pyx',
                                                  'src/MdisPlugin.cpp',
                                                  'src/MdisSensorModel.cpp']),
              generate_extension('usgscam.orex', ['usgscam/orex.pyx',
                                                  'src/ORexPlugin.cpp',
                                                  'src/ORexSensorModel.cpp']),
              generate_extension('usgscam.genericls', ['usgscam/genericls.pyx',
                                                       'src/UsgsAstroLsISD.cpp',
                                                       'src/UsgsAstroLsPlugin.cpp',
                                                       'src/UsgsAstroLsSensorModel.cpp',
                                                       'src/UsgsAstroLsStateData.cpp'])]

setup(
    name='usgscam',
    version='0.1.0',
    ext_modules=cythonize(extensions),
    description='Cython wrapper to the USGS MDIS Camera Model',
    author='Jay Laura',
    packages = find_packages(),
    install_requires=['cycsm'])
