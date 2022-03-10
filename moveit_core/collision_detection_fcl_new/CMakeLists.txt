set(MOVEIT_LIB_NAME moveit_collision_detection_fcl_new)

add_library(${MOVEIT_LIB_NAME}
  src/fcl_integration/fcl_utils.cpp
  src/fcl_integration/fcl_collision_object_wrapper.cpp
  src/fcl_integration/fcl_bvh_manager.cpp
  src/fcl_integration/fcl_discrete_bvh_manager.cpp
  src/collision_env_fcl_new.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")


target_link_libraries(${MOVEIT_LIB_NAME} moveit_collision_detection ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${LIBFCL_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(demo_box_box_fcl examples/box_box_example.cpp)
target_link_libraries(demo_box_box_fcl ${MOVEIT_LIB_NAME})

add_library(collision_detector_fcl_plugin_new src/collision_detector_fcl_plugin_loader_new.cpp)
set_target_properties(collision_detector_fcl_plugin_new PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(collision_detector_fcl_plugin_new ${catkin_LIBRARIES} ${MOVEIT_LIB_NAME} moveit_planning_scene)


install(TARGETS ${MOVEIT_LIB_NAME} collision_detector_fcl_plugin_new
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(FILES ../collision_detector_fcl_description_new.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
