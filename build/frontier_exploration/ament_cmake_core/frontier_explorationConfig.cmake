# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_frontier_exploration_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED frontier_exploration_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(frontier_exploration_FOUND FALSE)
  elseif(NOT frontier_exploration_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(frontier_exploration_FOUND FALSE)
  endif()
  return()
endif()
set(_frontier_exploration_CONFIG_INCLUDED TRUE)

# output package information
if(NOT frontier_exploration_FIND_QUIETLY)
  message(STATUS "Found frontier_exploration: 0.0.0 (${frontier_exploration_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'frontier_exploration' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT frontier_exploration_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(frontier_exploration_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${frontier_exploration_DIR}/${_extra}")
endforeach()
