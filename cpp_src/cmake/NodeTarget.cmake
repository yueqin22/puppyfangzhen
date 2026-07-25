set(NODE_TARGET_TEMPLATE_DIR "${CMAKE_CURRENT_LIST_DIR}")

function(add_ros_node_from_class target_name node_source header_path node_class)
  set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
  file(MAKE_DIRECTORY "${generated_dir}")
  set(main_source "${generated_dir}/${target_name}_main.cpp")
  configure_file(
    "${NODE_TARGET_TEMPLATE_DIR}/node_main.cpp.in"
    "${main_source}"
    @ONLY
  )

  add_executable(${target_name} ${node_source} "${main_source}")
  target_include_directories(${target_name} PRIVATE ${INCLUDE_ROOT})
  ament_target_dependencies(${target_name} ${ROS_DEPS})
endfunction()
