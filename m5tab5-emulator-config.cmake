
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was m5tab5-emulator-config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

# M5Stack Tab5 Emulator CMake Configuration

include("${CMAKE_CURRENT_LIST_DIR}/m5tab5-emulator-targets.cmake")

# Required dependencies
find_dependency(Threads REQUIRED)
find_dependency(PkgConfig REQUIRED)

# SDL2 dependency
find_dependency(SDL2)
if(NOT SDL2_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(SDL2 REQUIRED sdl2)
endif()

# Optional dependencies (fetched automatically if not found)
find_package(nlohmann_json QUIET)
find_package(spdlog QUIET)

check_required_components(m5tab5-emulator)
