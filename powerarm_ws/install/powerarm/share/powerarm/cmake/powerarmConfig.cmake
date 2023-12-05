# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_powerarm_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED powerarm_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(powerarm_FOUND FALSE)
  elseif(NOT powerarm_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(powerarm_FOUND FALSE)
  endif()
  return()
endif()
set(_powerarm_CONFIG_INCLUDED TRUE)

# output package information
if(NOT powerarm_FIND_QUIETLY)
  message(STATUS "Found powerarm: 0.0.0 (${powerarm_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'powerarm' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${powerarm_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(powerarm_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${powerarm_DIR}/${_extra}")
endforeach()
