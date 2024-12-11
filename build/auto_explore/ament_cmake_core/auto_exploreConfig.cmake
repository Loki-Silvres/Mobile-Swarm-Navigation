# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_auto_explore_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED auto_explore_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(auto_explore_FOUND FALSE)
  elseif(NOT auto_explore_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(auto_explore_FOUND FALSE)
  endif()
  return()
endif()
set(_auto_explore_CONFIG_INCLUDED TRUE)

# output package information
if(NOT auto_explore_FIND_QUIETLY)
  message(STATUS "Found auto_explore: 0.0.0 (${auto_explore_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'auto_explore' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${auto_explore_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(auto_explore_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${auto_explore_DIR}/${_extra}")
endforeach()
