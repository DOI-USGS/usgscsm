if (NOT TARGET gtest)
  set(GOOGLETEST_ROOT external/gtest/googletest CACHE STRING "Google Test source root")

  include_directories(SYSTEM
      ${PROJECT_SOURCE_DIR}/${GOOGLETEST_ROOT}
      ${PROJECT_SOURCE_DIR}/${GOOGLETEST_ROOT}/include
      )

  set(GOOGLETEST_SOURCES
      ${PROJECT_SOURCE_DIR}/${GOOGLETEST_ROOT}/src/gtest-all.cc
      ${PROJECT_SOURCE_DIR}/${GOOGLETEST_ROOT}/src/gtest_main.cc
      )

  # These are submodule sources, not generated ones. Marking them GENERATED used
  # to let configure succeed with an uninitialized submodule, turning a missing
  # checkout into an obscure "No rule to make target" at build time.
  if(NOT EXISTS ${PROJECT_SOURCE_DIR}/${GOOGLETEST_ROOT}/src/gtest-all.cc)
    message(FATAL_ERROR
      "Google Test sources not found at ${PROJECT_SOURCE_DIR}/${GOOGLETEST_ROOT}. "
      "Run: git submodule update --init --recursive "
      "(or configure with -DUSGSCSM_BUILD_TESTS=OFF)")
  endif()

  add_library(gtest ${GOOGLETEST_SOURCES})
endif()
