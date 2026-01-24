# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_fines_serial_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED fines_serial_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(fines_serial_FOUND FALSE)
  elseif(NOT fines_serial_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(fines_serial_FOUND FALSE)
  endif()
  return()
endif()
set(_fines_serial_CONFIG_INCLUDED TRUE)

# output package information
if(NOT fines_serial_FIND_QUIETLY)
  message(STATUS "Found fines_serial: 0.0.0 (${fines_serial_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'fines_serial' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${fines_serial_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(fines_serial_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${fines_serial_DIR}/${_extra}")
endforeach()
