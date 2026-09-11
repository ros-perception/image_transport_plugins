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
    pkg_check_modules(_SVT SvtAv1Dec)
endif()

find_path(SVTDEC_INCLUDE_DIR
          NAMES svt-av1/EbSvtAv1Dec.h
          PATHS ${_SVTDEC_INCLUDEDIR}
)
message(_SVTDEC_INCLUDEDIR ${_SVTDEC_INCLUDEDIR})

find_library(SVTDEC_LIBRARY
            NAMES SvtAv1Dec
            PATHS ${_SVTDEC_LIBDIR})

if(SVTDEC_LIBRARY)
    set(SVTDEC_LIBRARIES
        ${SVTDEC_LIBRARIES}
        ${SVTDEC_LIBRARY})
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SVTDEC
                                  FOUND_VAR SVTDEC_FOUND
                                  REQUIRED_VARS SVTDEC_LIBRARY SVTDEC_LIBRARIES SVTDEC_INCLUDE_DIR
                                  VERSION_VAR _SVTDEC_VERSION)

mark_as_advanced(SVTDEC_INCLUDE_DIR SVTDEC_LIBRARY SVTDEC_LIBRARIES)
