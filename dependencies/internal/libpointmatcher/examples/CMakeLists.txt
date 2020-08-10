add_executable(pmicp icp.cpp)
target_link_libraries(pmicp pointmatcher)
install(TARGETS pmicp RUNTIME DESTINATION bin)

add_executable(icp_simple icp_simple.cpp)
target_link_libraries(icp_simple pointmatcher)

add_executable(align_sequence align_sequence.cpp)
target_link_libraries(align_sequence pointmatcher)

add_executable(list_modules list_modules.cpp)
target_link_libraries(list_modules pointmatcher)

add_executable(build_map build_map.cpp)
target_link_libraries(build_map pointmatcher)

add_executable(compute_overlap compute_overlap.cpp)
target_link_libraries(compute_overlap pointmatcher)

add_executable(icp_advance_api icp_advance_api.cpp)
target_link_libraries(icp_advance_api pointmatcher)

add_executable(icp_customized icp_customized.cpp)
target_link_libraries(icp_customized pointmatcher)
