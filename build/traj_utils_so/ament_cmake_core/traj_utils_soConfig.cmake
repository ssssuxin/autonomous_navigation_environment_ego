# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_traj_utils_so_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED traj_utils_so_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(traj_utils_so_FOUND FALSE)
  elseif(NOT traj_utils_so_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(traj_utils_so_FOUND FALSE)
  endif()
  return()
endif()
set(_traj_utils_so_CONFIG_INCLUDED TRUE)

# output package information
if(NOT traj_utils_so_FIND_QUIETLY)
  message(STATUS "Found traj_utils_so: 0.0.0 (${traj_utils_so_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'traj_utils_so' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${traj_utils_so_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(traj_utils_so_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${traj_utils_so_DIR}/${_extra}")
endforeach()
