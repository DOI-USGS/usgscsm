# CSM-CameraModel

## Building with conda

Install conda if you do not already have it.
```bash
wget https://repo.continuum.io/miniconda/Miniconda3-latest-Linux-x86_64.sh -O miniconda.sh;
bash miniconda.sh -b
```
> You can add a `-p <install-prefix>` to choose where to install miniconda. By default, it will install it to `$HOME/miniconda3`.

### Setting up conda for bash
Copy and paste the following into a terminal running the `bash` shell:
```bash
echo -e "\n\n# Adding miniconda3 to PATH" >> $HOME/.bashrc && \
echo -e "export PATH=$HOME/miniconda3/bin:\$PATH" >> $HOME/.bashrc && \
source $HOME/.bashrc && \
which conda
```
> [bash installation reference](https://conda.io/docs/user-guide/install/linux.html "Reference to bash conda install")

### Setting up conda for tcsh
Copy and paste the following into a terminal running the `tcsh` shell:
```tcsh
echo  "\n\n# Setting up miniconda3 for tcsh" >> $HOME/.cshrc && \
echo  "source $HOME/miniconda3/etc/profile.d/conda.csh > /dev/null" >> $HOME/.cshrc && \
source $HOME/.cshrc && \
which conda
```
> [tcsh installation reference](https://github.com/ESMValGroup/ESMValTool/issues/301 "Reference to tcsh conda install")

### Creating an isolated conda environment
Run the following commands to create a self-contained dev environment for CSM-CameraModel (type `y` to confirm creation):
```bash
conda create -n csmdev -c usgs-astrogeology cmake libcsm
```

#### Activating the environment
After creating the `csmdev` environment and installing cmake and libcsm into it, we need to activate it. Right now, cmake, libcsm, and their dependencies are isolated to a conda environment and we need to tell conda that we want to use it. The activation command depends on your shell.
* **bash**: `source activate csmdev`
* **tcsh**: `conda activate csmdev`

After you've set up conda, you can build CSM-CameraModel:
```bash

```

---

## Building without a package manager
To build:

1. Install [libcsmapi](https://github.com/sminster/csm "CSM API")
    > You can install this with an `INSTDIR` of your choice, or let it default (see [libcsmapi README](https://github.com/sminster/csm/blob/master/README))
```bash
mkdir $HOME/csmenv
cd $HOME
git clone git@github.com:sminster/csm.git
cd csm
make -f Makefile.linux64 all install clean INSTDIR="$csmenv"
```
2. Install cmake >= 3.10
```bash
cd $HOME
wget https://cmake.org/files/v3.12/cmake-3.12.0-Linux-x86_64.tar.gz
tar xzf cmake-3.12.0-Linux-x86_64.tar.gz
rsync -azv cmake-3.12.0-Linux-x86_64/ $HOME/csmenv/
echo -e "\n#Prepending csm env to path\nsetenv PATH "$HOME/csmenv/bin:$PATH" >> $HOME/.cshrc
source $HOME/.cshrc
```
3. Fork and clone down this repo and its submodules (gtest)
```bash
git clone --recursive git@github.com:<username>/CSM-CameraModel.git
cd CSM-CameraModel
git remote add upstream git@github.com:USGS-Astrogeology/CSM-CameraModel.git
git pull upstream master
git submodule update --init --recursive
git push -u origin master
```
4. `mkdir build` && `cd build`
5. `cmake -DCSM_INCLUDE_DIR="${csmenv}/include/csm -DCSM_LIBRARY="${csmenv}/lib/libcsmapi.so .. && make`
