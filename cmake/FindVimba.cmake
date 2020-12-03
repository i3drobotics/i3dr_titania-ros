cmake_minimum_required(VERSION 2.6)

include(TargetArchitecture)
target_architecture(ARCH)

set (vimba_DIR ${PROJECT_SOURCE_DIR}/vimba)
if (NOT vimba_DIR)
    message( FATAL_ERROR "MUST define vimba_DIR" )
endif()

find_path(VIMBA_INCLUDE_DIR
    NAMES VimbaC/Include/VimbaC.h
    HINTS ${vimba_DIR}/VimbaSDK
    PATH_SUFFIXES include
    NO_DEFAULT_PATH
    DOC "The VIMBA include directory"
)

if (NOT VIMBA_INCLUDE_DIR)
    message(WARNING "include directory not found")
endif()

# Get library folder based on architecture
# Currently only supporting Linux x86 and x64
if("${ARCH}" STREQUAL i386)
    set(VIMBA_LIB_DIR ${vimba_DIR}/VimbaSDK/VimbaCPP/DynamicLib/x86_32bit/)
elseif("${ARCH}" STREQUAL x86_64)
    set(VIMBA_LIB_DIR ${vimba_DIR}/VimbaSDK/VimbaCPP/DynamicLib/x86_64bit/)
else()
    message(FATAL_ERROR "[Vimba]: Architecture ${ARCH} not suported. Exiting...")
endif()

find_library(VIMBA_C_LIBRARY 
    NAMES VimbaC
    PATHS ${VIMBA_LIB_DIR}
    NO_DEFAULT_PATH
    DOC "The VIMBA C library"
)

find_library(VIMBA_CPP_LIBRARY 
    NAMES VimbaCPP
    PATHS ${VIMBA_LIB_DIR}
    NO_DEFAULT_PATH
    DOC "The VIMBA C library"
)

if (NOT VIMBA_C_LIBRARY)
    message(WARNING "vimba C library directory not found")
endif()

if (NOT VIMBA_CPP_LIBRARY)
    message(WARNING "vimba CPP library directory not found")
endif()

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LOGGING_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(VIMBA DEFAULT_MSG VIMBA_INCLUDE_DIR VIMBA_C_LIBRARY VIMBA_CPP_LIBRARY)

if (VIMBA_FOUND)
    set(vimba_LIBRARIES ${VIMBA_C_LIBRARY} ${VIMBA_CPP_LIBRARY})
    set(vimba_INCLUDE_DIRS ${VIMBA_INCLUDE_DIR} )
    set(vimba_DEFINITIONS )
else(VIMBA_FOUND)
    message(WARNING "VIMBA not found")
endif()

# Tell cmake GUIs to ignore the "local" variables.
mark_as_advanced(vimba_DIR VIMBA_LIBRARY VIMBA_INCLUDE_DIR) 