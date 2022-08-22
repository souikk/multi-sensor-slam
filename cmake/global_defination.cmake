
set(WORKSPACE_PATH ${PROJECT_SOURCE_DIR})
configure_file(${PROJECT_SOURCE_DIR}/include/localization/global_path/global_path.h.in ${PROJECT_BINARY_DIR}/include/global_path.h)
  
include_directories(${PROJECT_BINARY_DIR}/include)