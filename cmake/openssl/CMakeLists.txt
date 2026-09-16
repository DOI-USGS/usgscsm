# Static OpenSSL, built from the pristine submodule at external/openssl so it is
# baked into libusgscsm. STARDS needs libcrypto everywhere (S3 SigV4 signing);
# libssl only on Linux, where curl has no OS TLS backend to use instead.

cmake_minimum_required(VERSION 3.11)  # ExternalProject + BUILD_BYPRODUCTS
include(ExternalProject)

set(OPENSSL_SRC_DIR "${CMAKE_SOURCE_DIR}/external/openssl")

if(NOT EXISTS "${OPENSSL_SRC_DIR}/Configure")
  message(FATAL_ERROR
    "OpenSSL submodule not found at ${OPENSSL_SRC_DIR}. "
    "Run: git submodule update --init --recursive")
endif()

set(OPENSSL_PREFIX  "${CMAKE_CURRENT_BINARY_DIR}/openssl")
set(OPENSSL_BUILD   "${OPENSSL_PREFIX}/build")
set(OPENSSL_INSTALL "${OPENSSL_PREFIX}/install")
set(OPENSSL_INCLUDE_DIR "${OPENSSL_INSTALL}/include")

# Static archive extension differs by toolchain: MSVC produces .lib, Unix .a.
# Named once here; the archive file names and the make targets below both use it.
if(WIN32)
  set(OPENSSL_LIB_SUFFIX ".lib")
else()
  set(OPENSSL_LIB_SUFFIX ".a")
endif()
set(OPENSSL_CRYPTO_LIBNAME "libcrypto${OPENSSL_LIB_SUFFIX}")
set(OPENSSL_SSL_LIBNAME    "libssl${OPENSSL_LIB_SUFFIX}")
set(OPENSSL_CRYPTO_LIB "${OPENSSL_INSTALL}/lib/${OPENSSL_CRYPTO_LIBNAME}")
set(OPENSSL_SSL_LIB    "${OPENSSL_INSTALL}/lib/${OPENSSL_SSL_LIBNAME}")

# Pick the OpenSSL Configure target for the platform + architecture. OpenSSL
# builds one architecture per pass, so honor CMAKE_OSX_ARCHITECTURES when it
# names a single arch and reject a universal-binary request outright.
if(APPLE)
  set(OPENSSL_APPLE_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
  if(CMAKE_OSX_ARCHITECTURES)
    list(LENGTH CMAKE_OSX_ARCHITECTURES _osx_arch_count)
    if(_osx_arch_count GREATER 1)
      message(FATAL_ERROR
        "CMAKE_OSX_ARCHITECTURES=${CMAKE_OSX_ARCHITECTURES}: OpenSSL cannot be "
        "built as a universal binary in one pass. Configure one architecture, "
        "or build with -DUSGSCSM_ENABLE_STARDS=OFF to skip OpenSSL entirely.")
    endif()
    set(OPENSSL_APPLE_ARCH "${CMAKE_OSX_ARCHITECTURES}")
  endif()
  if(OPENSSL_APPLE_ARCH MATCHES "arm64|aarch64")
    set(OPENSSL_TARGET darwin64-arm64-cc)
  else()
    set(OPENSSL_TARGET darwin64-x86_64-cc)
  endif()
elseif(WIN32)
  # VC-WIN64A = 64-bit MSVC. Configure needs perl, which the runners have.
  set(OPENSSL_TARGET VC-WIN64A)
else()
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|aarch64")
    set(OPENSSL_TARGET linux-aarch64)
  else()
    set(OPENSSL_TARGET linux-x86_64)
  endif()
endif()

# nmake on Windows, make elsewhere. no-asm avoids a NASM dependency; the
# SHA256/HMAC STARDS needs work fine without the asm fast paths.
if(WIN32)
  set(OPENSSL_MAKE nmake)
  set(OPENSSL_ASM_ARG no-asm)
else()
  # ExternalProject cannot forward the generator's job server to a plain make,
  # so pick a level: the user's if set, else one job per core.
  if(DEFINED ENV{CMAKE_BUILD_PARALLEL_LEVEL})
    set(OPENSSL_JOBS "$ENV{CMAKE_BUILD_PARALLEL_LEVEL}")
  else()
    include(ProcessorCount)
    ProcessorCount(OPENSSL_JOBS)
    if(OPENSSL_JOBS EQUAL 0)
      set(OPENSSL_JOBS 1)
    endif()
  endif()
  set(OPENSSL_MAKE make -j${OPENSSL_JOBS})
  set(OPENSSL_ASM_ARG)  # empty: keep asm on Unix
endif()

# Naming the archives as make targets builds exactly those. There is no "just
# libcrypto" configure flag (no-ssl only disables protocol negotiation).
if(APPLE OR WIN32)
  set(OPENSSL_BUILD_LIBS ${OPENSSL_MAKE} ${OPENSSL_CRYPTO_LIBNAME})
  set(OPENSSL_BYPRODUCTS "${OPENSSL_CRYPTO_LIB}")
else()
  set(OPENSSL_BUILD_LIBS ${OPENSSL_MAKE} ${OPENSSL_SSL_LIBNAME} ${OPENSSL_CRYPTO_LIBNAME})
  set(OPENSSL_BYPRODUCTS "${OPENSSL_CRYPTO_LIB}" "${OPENSSL_SSL_LIB}")
endif()

# openssldir is compiled in as the runtime CA-store and config location, so it
# must name a path that exists on the *user's* machine, not in this build tree.
if(WIN32)
  set(OPENSSL_DIR_ARG)
else()
  set(OPENSSL_DIR_ARG --openssldir=/etc/ssl)
endif()

# build_generated must run before the object files, or generated headers
# (opensslv.h, ...) are missing. Configure runs out-of-tree so the submodule stays
# pristine, and through perl explicitly so Windows invokes it like Unix does.
ExternalProject_Add(openssl_ep
  SOURCE_DIR      "${OPENSSL_SRC_DIR}"
  BINARY_DIR      "${OPENSSL_BUILD}"
  # --libdir=lib: OpenSSL otherwise defaults to lib64 on many 64-bit Linux
  # distros, breaking the hardcoded lib/ paths above.
  CONFIGURE_COMMAND
    perl ${OPENSSL_SRC_DIR}/Configure ${OPENSSL_TARGET} no-shared no-tests no-docs no-apps
      no-legacy no-engine ${OPENSSL_ASM_ARG}
      --prefix=${OPENSSL_INSTALL} ${OPENSSL_DIR_ARG}
      --libdir=lib
  BUILD_COMMAND   ${OPENSSL_MAKE} build_generated COMMAND ${OPENSSL_BUILD_LIBS}
  INSTALL_COMMAND ${OPENSSL_MAKE} install_dev
  BUILD_BYPRODUCTS ${OPENSSL_BYPRODUCTS}
  LOG_CONFIGURE 1 LOG_BUILD 1 LOG_INSTALL 1
)

# Make the include dir exist at configure time so INTERFACE usage validates.
file(MAKE_DIRECTORY "${OPENSSL_INCLUDE_DIR}")

# libcrypto, needed on all platforms for S3 signing.
add_library(usgscsm_crypto STATIC IMPORTED GLOBAL)
set_target_properties(usgscsm_crypto PROPERTIES
    IMPORTED_LOCATION "${OPENSSL_CRYPTO_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${OPENSSL_INCLUDE_DIR}")
add_dependencies(usgscsm_crypto openssl_ep)

# System libraries static libcrypto's symbols need at the final link. macOS gets
# these from libSystem, so nothing is added there.
if(WIN32)
  set_target_properties(usgscsm_crypto PROPERTIES
      INTERFACE_LINK_LIBRARIES "ws2_32;gdi32;advapi32;crypt32;user32")
elseif(NOT APPLE)
  set(THREADS_PREFER_PTHREAD_FLAG ON)
  find_package(Threads REQUIRED)
  set_target_properties(usgscsm_crypto PROPERTIES
      INTERFACE_LINK_LIBRARIES "Threads::Threads;${CMAKE_DL_LIBS}")
endif()

# libssl is built and exposed only on Linux, where curl links OpenSSL for TLS.
if(NOT APPLE AND NOT WIN32)
  add_library(usgscsm_ssl STATIC IMPORTED GLOBAL)
  set_target_properties(usgscsm_ssl PROPERTIES
      IMPORTED_LOCATION "${OPENSSL_SSL_LIB}"
      INTERFACE_INCLUDE_DIRECTORIES "${OPENSSL_INCLUDE_DIR}")
  add_dependencies(usgscsm_ssl openssl_ep)
endif()

# Expose the install paths to the parent scope so curl can use this OpenSSL.
set(USGSCSM_OPENSSL_INSTALL "${OPENSSL_INSTALL}" PARENT_SCOPE)
set(USGSCSM_OPENSSL_INCLUDE_DIR "${OPENSSL_INCLUDE_DIR}" PARENT_SCOPE)
set(USGSCSM_OPENSSL_CRYPTO_LIB "${OPENSSL_CRYPTO_LIB}" PARENT_SCOPE)
set(USGSCSM_OPENSSL_SSL_LIB "${OPENSSL_SSL_LIB}" PARENT_SCOPE)
