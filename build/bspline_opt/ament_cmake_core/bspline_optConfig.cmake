# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_bspline_opt_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED bspline_opt_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(bspline_opt_FOUND FALSE)
  elseif(NOT bspline_opt_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(bspline_opt_FOUND FALSE)
  endif()
  return()
endif()
set(_bspline_opt_CONFIG_INCLUDED TRUE)

# output package information
if(NOT bspline_opt_FIND_QUIETLY)
  message(STATUS "Found bspline_opt: 0.0.0 (${bspline_opt_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'bspline_opt' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${bspline_opt_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(bspline_opt_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${bspline_opt_DIR}/${_extra}")
endforeach()
