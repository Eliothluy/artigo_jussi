check_include_file_cxx(stdint.h HAVE_STDINT_H)
if(HAVE_STDINT_H)
    add_definitions(-DHAVE_STDINT_H)
endif()

find_external_library(DEPENDENCY_NAME e2sim
                      HEADER_NAME e2sim.hpp
                      LIBRARY_NAME e2sim
                      SEARCH_PATHS /usr/local/include/e2sim)

if(NOT ${e2sim_FOUND})
    message(WARNING "e2sim is required by nori and was not found")
    return()
endif()

include_directories(${e2sim_INCLUDE_DIRS})
message(STATUS "NORI: e2sim dirs: ${e2sim_INCLUDE_DIRS}")
message(STATUS "NORI: e2sim libs: ${e2sim_LIBRARIES}")

build_lib(
    LIBNAME nori
    SOURCE_FILES
        model/E2-interface-slicing.cc
        helper/E2-term-helper-slicing.cc
    HEADER_FILES
        model/E2-interface-slicing.h
        helper/E2-term-helper-slicing.h
    LIBRARIES_TO_LINK
        ${libnr}
        ${liboran-interface}
        ${e2sim_LIBRARIES}
        ${libcore}
)
