# Find TurboJPEG
#
# The following are set after configuration is done:
#  TurboJPEG_FOUND
#  TurboJPEG_INCLUDE_DIRS
#  TurboJPEG_LIBRARIES

include(FindPackageHandleStandardArgs)

find_library(TurboJPEG_LIBRARY turbojpeg)
find_path(TurboJPEG_INCLUDE_DIR turbojpeg.h)

find_package_handle_standard_args(TurboJPEG
  REQUIRED_VARS TurboJPEG_LIBRARY TurboJPEG_INCLUDE_DIR
)

if(TurboJPEG_FOUND)
  set(TurboJPEG_INCLUDE_DIRS "${TurboJPEG_INCLUDE_DIR}")
  set(TurboJPEG_LIBRARIES "${TurboJPEG_LIBRARY}")
endif()
