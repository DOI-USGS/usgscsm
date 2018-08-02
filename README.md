# CSM-CameraModel

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
wget https://cmake.org/files/v3.10/cmake-3.10.0-Linux-x86_64.tar.gz
tar xzf cmake-3.10.0-Linux-x86_64.tar.gz
rsync -azv cmake-3.10.0-Linux-x86_64/ $HOME/csmenv/
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
