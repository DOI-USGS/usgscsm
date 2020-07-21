mkdir build
cd build
cmake -G "Visual Studio 15 2017 Win64" -DUSGSCSM_EXTERNAL_DEPS=ON -DBUILD_TESTS=OFF ..
cmake --build . --target ALL_BUILD --config Release
copy Release\usgscsm.dll %LIBRARY_BIN%

if errorlevel 1 exit 1
