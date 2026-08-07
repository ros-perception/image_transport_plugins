# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(_SVT SvtAv1Enc)
endif()

find_path(SVTENC_INCLUDE_DIR
          NAMES svt-av1/EbSvtAv1Enc.h
          PATHS ${_SVTENC_INCLUDEDIR}
)
message(_SVTENC_INCLUDEDIR ${_SVTENC_INCLUDEDIR})

find_library(SVTENC_LIBRARY
            NAMES SvtAv1Enc
            PATHS ${_SVTENC_LIBDIR})

if(SVTENC_LIBRARY)
    set(SVTENC_LIBRARIES
        ${SVTENC_LIBRARIES}
        ${SVTENC_LIBRARY})
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SVTENC
                                  FOUND_VAR SVTENC_FOUND
                                  REQUIRED_VARS SVTENC_LIBRARY SVTENC_LIBRARIES SVTENC_INCLUDE_DIR
                                  VERSION_VAR _SVTENC_VERSION)

mark_as_advanced(SVTENC_INCLUDE_DIR SVTENC_LIBRARY SVTENC_LIBRARIES)
