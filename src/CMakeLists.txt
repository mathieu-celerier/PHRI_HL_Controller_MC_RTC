set(controller_SRC
  PHRI_HLController.cpp
  PPCTask.cpp
  MCGamePiece.cpp
  MCGameState.cpp
)

set(controller_HDR
  ${CMAKE_CURRENT_SOURCE_DIR}/../include/hl_controller/PHRI_HLController.h
  ${CMAKE_CURRENT_SOURCE_DIR}/../include/hl_controller/PPCTask.h
  ${CMAKE_CURRENT_SOURCE_DIR}/../include/hl_controller/MCGameState.h
  ${CMAKE_CURRENT_SOURCE_DIR}/../include/hl_controller/MCGamePiece.h
  ${CMAKE_CURRENT_SOURCE_DIR}/../include/hl_controller/utils/eigen_utils.h
  ${CMAKE_CURRENT_SOURCE_DIR}/../include/hl_controller/utils/ROSSubscriber.h
)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DPHRI_HLController_EXPORTS")
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_control_fsm mc_rtc::mc_rtc_ros)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION "${MC_RTC_LIBDIR}"
  LIBRARY DESTINATION "${MC_RTC_LIBDIR}"
  RUNTIME DESTINATION bin)

add_controller(${PROJECT_NAME}_controller lib.cpp "")
target_link_libraries(${PROJECT_NAME}_controller PUBLIC ${PROJECT_NAME})

add_subdirectory(states)