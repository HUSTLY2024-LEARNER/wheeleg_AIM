# 设置要复制的文件列表
cmake_minimum_required(VERSION 3.5)
project(utilis)
set(MODEL_FILES
    models/armor_yolo_x.bin
    models/armor_yolo_x.xml
)

# 将文件复制到构建目录
foreach(file ${MODEL_FILES})
    file(COPY ${file} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/${file})
endforeach()

