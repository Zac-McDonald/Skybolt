# This module defines
#  cigicl_FOUND, if false, do not try to link
#  cigicl_LIBRARIES, libraries to link against
#  cigicl_INCLUDE_DIR, where to find headers

SET (opendis_INCLUDE_DIR "W:/Prograda/ThirdParty/open-dis-cpp/src") # Would much rather this alias to opendis/**/*.h
SET (opendis_LIBRARIES "W:/Prograda/ThirdParty/open-dis-cpp/build/Debug/OpenDIS6.lib")

IF(opendis_LIBRARIES AND opendis_INCLUDE_DIR)
  SET(opendis_FOUND "YES")
ENDIF()
