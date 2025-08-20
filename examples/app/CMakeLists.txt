include_directories(${CMAKE_SOURCE_DIR}/include)

set(exeNames "clientPNC" )

foreach(exeName IN LISTS exeNames)
    add_executable(${exeName} ${exeName}.cpp)

    target_link_libraries(${exeName} utils costmap pathPlanner localPlanner loader json comm)

    install(TARGETS ${exeName} DESTINATION ${INSTALL_BIN_DIR})
endforeach()

