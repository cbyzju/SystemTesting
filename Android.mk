LOCAL_PATH :=$(call my-dir)

include $(CLEAR_VARS)
OpenCV_INSTALL_MODULES:=on
OPENCV_CAMERA_MODULES:=off
APP_STL := gnustl_static
OPENCV_LIB_TYPE:=STATIC

#ifeq ("$(wildcard $(OPENCV_MK_PATH))","")
include C:/OpenCV-2.4.10-android-sdk/sdk/native/jni/OpenCV.mk
#else
#include $(OPENCV_MK_PATH)
#endif

LOCAL_MODULE :=jni-input
LOCAL_CFLAGS =  -DANDROID_NDK_BUILD -D__STDC_FORMAT_MACROS -D__STDC_INT64__
LOCAL_LDLIBS +=  -llog -ldl
LOCAL_LDLIBS += -L$(SYSROOT)/usr/lib -llog


LOCAL_SRC_FILES :=com_orbbec_NativeNI_NativeGestureDetect.cpp\
                  Htime.cpp\
                  projectorCamera.cpp\
                  sendEvent.cpp\
                  json_reader.cpp\
                  json_value.cpp\
                  json_writer.cpp\
                  touchInfo.cpp\
                  state.cpp\
                  singleActive.cpp\
                  role.cpp\
                  idle.cpp\
                  doubleTwoHandActive.cpp\
                  doubleOneHandActive.cpp


include $(BUILD_SHARED_LIBRARY)