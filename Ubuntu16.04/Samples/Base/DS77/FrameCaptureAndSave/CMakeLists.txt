project(FrameCaptureAndSave)
add_executable( ${PROJECT_NAME} FrameCaptureAndSave.cpp)
target_link_libraries(${PROJECT_NAME} Nebula_api DSImgPreProcess)