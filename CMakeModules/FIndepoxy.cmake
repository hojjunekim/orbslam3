# Copyright (C) 2024 Igalia S.L.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1.  Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
# 2.  Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND ITS CONTRIBUTORS ``AS
# IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR ITS
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#[=======================================================================[.rst:
FindEpoxy
---------

Find the Epoxy (*libepoxy*) headers and library.

Imported Targets
^^^^^^^^^^^^^^^^

``Epoxy::Epoxy``
  The Epoxy library, if found.

Result Variables
^^^^^^^^^^^^^^^^

This will define the following variables in your project:

``Epoxy_FOUND``
  true if (the requested version of) Epoxy is available.
``Epoxy_VERSION``
  the version of Epoxy; only defined when a ``pkg-config`` module for
  Epoxy is available.
``Epoxy_LIBRARIES``
  the libraries to link against to use Epoxy.
``Epoxy_INCLUDE_DIRS``
  where to find the Epoxy headers.
``Epoxy_COMPILE_OPTIONS``
  this should be passed to target_compile_options(), if the
  target is not used for linking

#]=======================================================================]

find_package(PkgConfig QUIET)
pkg_check_modules(PC_epoxy QUIET epoxy)
set(epoxy_COMPILE_OPTIONS ${PC_epoxy_CFLAGS_OTHER})
set(epoxy_VERSION ${PC_epoxy_VERSION})

find_path(epoxy_INCLUDE_DIR
    NAMES epoxy/common.h
    HINTS ${PC_epoxy_INCLUDEDIR} ${PC_epoxy_INCLUDE_DIR}
)

find_library(epoxy_LIBRARY
    NAMES ${epoxy_NAMES} epoxy
    HINTS ${PC_epoxy_LIBDIR} ${PC_epoxy_LIBRARY_DIRS}
)

# The libepoxy headers do not have anything usable to detect its version
if ("${epoxy_FIND_VERSION}" VERSION_GREATER "${epoxy_VERSION}")
    if (epoxy_VERSION)
        message(FATAL_ERROR "Required version (${epoxy_FIND_VERSION}) is"
            " higher than found version (${epoxy_VERSION})")
    else ()
        message(WARNING "Cannot determine epoxy version withouit pkg-config")
    endif ()
endif ()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(epoxy
    FOUND_VAR epoxy_FOUND
    REQUIRED_VARS epoxy_LIBRARY
    VERSION_VAR epoxy_VERSION
)

if (epoxy_LIBRARY AND NOT TARGET epoxy::epoxy)
    add_library(epoxy::epoxy UNKNOWN IMPORTED GLOBAL)
    set_target_properties(epoxy::epoxy PROPERTIES
        IMPORTED_LOCATION "${epoxy_LIBRARY}"
        INTERFACE_COMPILE_OPTIONS "${epoxy_COMPILE_OPTIONS}"
        INTERFACE_INCLUDE_DIRECTORIES "${epoxy_INCLUDE_DIR}"
    )
endif ()

mark_as_advanced(
    epoxy_LIBRARY
    epoxy_INCLUDE_DIR
)

if (epoxy_FOUND)
    set(epoxy_LIBRARIES ${epoxy_LIBRARY})
    set(epoxy_INCLUDE_DIRS ${epoxy_INCLUDE_DIR})
endif ()