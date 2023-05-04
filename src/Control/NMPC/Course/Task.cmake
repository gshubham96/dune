##########################################################################
# CASADI AND IPOPT LIBS                                                           #
##########################################################################
# for using casadi
find_package(casadi REQUIRED)
# set the Ipopt library and include directory
set(LIBRARY_DIRS /usr/local/lib)
if(UNIX AND NOT APPLE)
    set(IPOPT_INCLUDE_DIRS /usr/include/coin)
elseif(APPLE)
    set(IPOPT_INCLUDE_DIRS /usr/local/include/coin-or)
endif()

# set casadi include directory
set(CASADI_INCLUDE_DIR /usr/local/include/casadi)

# find casadi library
find_library(CASADI_LIBRARY
    NAMES casadi
    HINTS ${CASADI_INCLUDE_DIR}/../lib $ENV{CASADI_PREFIX}/lib)
if(CASADI_LIBRARY)
    set(CASADI_LIBRARIES ${CASADI_LIBRARIES} ${CASADI_LIBRARY})
endif()

# installing system dir
include_directories(
    SYSTEM ${IPOPT_INCLUDE_DIRS}
    SYSTEM ${CASADI_INCLUDE_DIR}
)

# library directories
link_directories(${LIBRARY_DIRS})

# find all the header files
# file(GLOB HEADER_FILES_HPP ${CMAKE_SOURCE_DIR}/include/*.hpp)
# file(GLOB HEADER_FILES_H ${CMAKE_SOURCE_DIR}/include/*.h)
