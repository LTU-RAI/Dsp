include(LibFindMacros)

# Dependencies
#libfind_package(dsl)

# Use pkg-config to get hints about paths
libfind_pkg_check_modules(DSL_PKGCONF dsl)

# Include dir
find_path(DSL_INCLUDE_DIR
  NAMES search.h
  PATHS ${DSL_PKGCONF_INCLUDE_DIRS}
)

# Finally the library itself
find_library(DSL_LIBRARY
  NAMES dsl
  PATHS ${DSL_PKGCONF_LIBRARY_DIRS}
)

# Set the include dir variables and the libraries and let libfind_process do the rest.
# NOTE: Singular variables for this library, plural for libraries this this lib depends on.
set(DSL_PROCESS_INCLUDES DSL_INCLUDE_DIR)
set(DSL_PROCESS_LIBS DSL_LIBRARY)
libfind_process(DSL)