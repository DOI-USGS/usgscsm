Creating an environment
-----------------------
.. code-block:: bash

    conda create -n csm python=3.6
    conda activate csm
    conda install -c conda-forge csmapi
    conda env update -n csm -f environment.yml

Due to version incompatibilities, setting up a full environment can be somewhat
difficult to navigate, and it is recommended that you follow these steps.  This
guide assumes that you already have anaconda set up on your computer.  If you do
not, please follow `these instructions <https://github.com/conda-forge/miniforge>`_.
