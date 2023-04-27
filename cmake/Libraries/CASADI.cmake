############################################################################

############################################################################

############################################################################
# Author: Shubham Garg                                                  #
############################################################################

if(CASADI)
  find_library(CASADI_LIBRARY NAMES casadi PATHS /usr/local/lib)

  if(${CASADI_LIBRARY} STRLESS "libcasadi.so")
    # Casadi Present
    set(DUNE_SYS_HAS_CASADI 1 CACHE INTERNAL "Casadi library")
    set(DUNE_USING_CASADI 1 CACHE INTERNAL "Casadi library")

    # FIND_PACKAGE(Casadi REQUIRED)
    dune_add_lib(casadi)

    # Check Header
    dune_test_header(casadi/casadi.hpp)

  else()
    # Casadi not found on the system.
    message(SEND_ERROR "Casadi was not found on the system.")
    set(DUNE_SYS_HAS_CASADI 0 CACHE INTERNAL "Casadi library")
    set(DUNE_USING_CASADI 0 CACHE INTERNAL "Casadi library")

  endif()

endif(CASADI)
