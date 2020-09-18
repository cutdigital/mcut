# Originally copied from the MPPP project repository (12 Sept 2020):
# https://raw.githubusercontent.com/bluescarni/mppp/master/cmake/Findmp%2B%2B_MPIR.cmake

# Copyright (c) 2006, Laurent Montel, <montel@kde.org>
# Copyright (c) 2008-2020 Francesco Biscani, <bluescarni@gmail.com>

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. The name of the author may not be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
# NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ------------------------------------------------------------------------------------------

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
if(mcut_MPIR_FOUND AND NOT TARGET mcut::MPIR)
    add_library(mcut::MPIR UNKNOWN IMPORTED)
    set_target_properties(mcut::MPIR PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${MCUT_MPIR_INCLUDE_DIR}"
        IMPORTED_LINK_INTERFACE_LANGUAGES "C" IMPORTED_LOCATION "${MCUT_MPIR_LIBRARY}")
endif()
