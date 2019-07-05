cmake_minimum_required(VERSION 3.13)
project(axidma_api)

set(CMAKE_CXX_STANDARD 14)
set(SRC ./src)

add_library(axidma_api ${SRC}/AxiDMA.cpp ${SRC}/AxiDMA.h ${SRC}/AxiDmaBuffer.cpp
            ${SRC}/AxiDmaBuffer.h ${SRC}/AxiDmaDescriptors.cpp
            ${SRC}/AxiDmaDescriptors.h)
