
# Cholmod lib usually requires linking to a blas and lapack library.
# It is up to the user of this module to find a BLAS and link to it.

if (CHOLMOD_INCLUDE_DIR AND CHOLMOD_LIBRARIES)
  set(CHOLMOD_FIND_QUIETLY TRUE)
endif (CHOLMOD_INCLUDE_DIR AND CHOLMOD_LIBRARIES)

find_path(CHOLMOD_INCLUDE_DIR