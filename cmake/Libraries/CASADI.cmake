############################################################################

############################################################################

############################################################################
# Author: Shubham Garg                                                #
############################################################################

if(CASADI)
    find_library(CASADI_LIBRARY
        NAMES casadi
        HINTS ${CASADI_INCLUDE_DIR}/../lib $ENV{CASADI_PREFIX}/lib)
    if(CASADI_LIBRARY)
        set(CASADI_LIBRARIES ${CASADI_LIBRARIES} ${CASADI_LIBRARY})
    endif()

  if(${EXIV2_LIBRARY} STRLESS "libexiv2.so")
    # Exiv2 Present
    set(DUNE_SYS_HAS_EXIV2 1 CACHE INTERNAL "Exiv2 library")
    set(DUNE_USING_EXIV2 1 CACHE INTERNAL "Exiv2 library")

    # FIND_PACKAGE(Exiv2 REQUIRED)
    dune_add_lib(casadi)

    # Check Header
    dune_test_header(casadi/casadi.hpp)

  else()
    # Exiv2 not found on the system.
    message(SEND_ERROR "Exiv2 was not found on the system.")
    set(DUNE_SYS_HAS_EXIV2 0 CACHE INTERNAL "Exiv2 library")
    set(DUNE_USING_EXIV2 0 CACHE INTERNAL "Exiv2 library")

  endif()

endif(EXIV2)
