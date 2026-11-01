cmake_minimum_required(VERSION 3.16)
project(rml63ii_eg2_4c2_gazebo)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(TBB REQUIRED)

add_executable(gazebo_stop src/gazebo_stop.cpp)
target_include_directories(gazebo_stop PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_directories(gazebo_stop PRIVATE ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(gazebo_stop ${GAZEBO_LIBRARIES} TBB::tbb)
target_compile_features(gazebo_stop PRIVATE cxx_std_17)
install(TARGETS gazebo_stop DESTINATION lib/${PROJECT_NAME})

foreach(asset_dir urdf mujoco assets config model_mjcf model_source model_gazebo worlds launch scripts)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${asset_dir}")
    install(DIRECTORY ${asset_dir} DESTINATION share/${PROJECT_NAME}
      PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
  endif()
endforeach()
install(PROGRAMS scripts/arm_control.py scripts/entity_tf.py DESTINATION lib/${PROJECT_NAME})
ament_package()
