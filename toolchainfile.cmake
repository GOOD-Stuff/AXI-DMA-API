SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_VERSION 1)
SET(CMAKE_SYSTEM_PROCESSOR arm)

# specify the cross compiler
SET(CROSS_PATH /home/vldmr/Programs/Xilinx/SDK/2018.2/gnu/aarch32/lin/gcc-arm-linux-gnueabi/bin)
SET(CMAKE_C_COMPILER ${CROSS_PATH}/arm-linux-gnueabihf-gcc)
SET(CMAKE_CXX_COMPILER ${CROSS_PATH}/arm-linux-gnueabihf-g++)
SET(CMAKE_AR ${CROSS_PATH}/arm-linux-gnueabihf-ar)
SET(CMAKE_STRIP ${CROSS_PATH}/arm-linux-gnueabihf-strip)

# where is the target environment
SET(CMAKE_FIND_ROOT_PATH  /home/vldmr/Programs/Xilinx/SDK/2018.2/gnu/aarch32/lin/gcc-arm-linux-gnueabi/arm-linux-gnueabihf/)

SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
SET(SYSTEM_USR_DIR ${CMAKE_FIND_ROOT_PATH})
