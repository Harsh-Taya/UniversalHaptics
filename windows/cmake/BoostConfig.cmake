# Minimal Boost package for building against the SOFA v26.06 Windows binaries.
# SOFA only needs Boost's header files (target Boost::boost); no compiled Boost libraries are used.
# Pass -DBOOST_INCLUDE_DIR=<folder that contains the "boost" folder> (e.g. C:/deps/boost_1_74_0).
if(NOT BOOST_INCLUDE_DIR OR NOT EXISTS "${BOOST_INCLUDE_DIR}/boost/version.hpp")
  message(FATAL_ERROR "BoostConfig.cmake: set BOOST_INCLUDE_DIR to the folder that contains boost/version.hpp")
endif()
set(Boost_VERSION 1.74.0)
set(Boost_VERSION_STRING 1.74.0)
set(Boost_INCLUDE_DIRS "${BOOST_INCLUDE_DIR}")
set(Boost_INCLUDE_DIR "${BOOST_INCLUDE_DIR}")
if(NOT TARGET Boost::headers)
  add_library(Boost::headers INTERFACE IMPORTED)
  set_target_properties(Boost::headers PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}"
    INTERFACE_COMPILE_DEFINITIONS "BOOST_ALL_NO_LIB")   # header-only: stop MSVC auto-linking Boost .lib files
endif()
if(NOT TARGET Boost::boost)
  add_library(Boost::boost INTERFACE IMPORTED)
  set_target_properties(Boost::boost PROPERTIES INTERFACE_LINK_LIBRARIES Boost::headers)
endif()
set(Boost_FOUND TRUE)
