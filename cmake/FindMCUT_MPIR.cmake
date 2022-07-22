#
# Copyright (c) 2021-2022 Floyd M. Chitalu.
# All rights reserved.
# 
# NOTE: This file is licensed under GPL-3.0-or-later (default). 
# A commercial license can be purchased from Floyd M. Chitalu. 
#  
# License details:
# 
# (A)  GNU General Public License ("GPL"); a copy of which you should have 
#      recieved with this file.
# 	    - see also: <http://www.gnu.org/licenses/>
# (B)  Commercial license.
#      - email: floyd.m.chitalu@gmail.com
# 
# The commercial license options is for users that wish to use MCUT in 
# their products for comercial purposes but do not wish to release their 
# software products under the GPL license. 
# 
# Author(s)     : Floyd M. Chitalu
#

if(MCUT_MPIR_INCLUDE_DIR AND MCUT_MPIR_LIBRARY)
    # Already in cache, be silent
    set(mcut_MPIR_FIND_QUIETLY TRUE)
endif()

find_path(MCUT_MPIR_INCLUDE_DIR NAMES mpir.h gmp.h)
find_library(MCUT_MPIR_LIBRARY NAMES mpir gmp)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(MCUT_MPIR DEFAULT_MSG MCUT_MPIR_INCLUDE_DIR MCUT_MPIR_LIBRARY)

mark_as_advanced(MCUT_MPIR_INCLUDE_DIR MCUT_MPIR_LIBRARY)

# NOTE: this has been adapted from CMake's FindPNG.cmake.
if(mcut_MPIR_FOUND AND NOT TARGET MPIR)
    add_library(MPIR UNKNOWN IMPORTED)
    set_target_properties(MPIR PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${MCUT_MPIR_INCLUDE_DIR}"
        IMPORTED_LINK_INTERFACE_LANGUAGES "C" IMPORTED_LOCATION "${MCUT_MPIR_LIBRARY}")
endif()
