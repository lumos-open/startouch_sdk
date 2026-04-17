cmake_minimum_required(VERSION 3.16)
project(user_app VERSION 1.0.0)


# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED CONFIG)
find_package(Threads REQUIRED)


# if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
#     message(STATUS "Target architecture is ARM")
#     set(DEPEND_LIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_r5_src/libarx_r5_src-arm64.so")
# else ()
#     set(DEPEND_LIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_r5_src/libarx_r5_src.so")
# endif ()

# target_include_directories(kinematic_solver PUBLIC lib/)
# target_link_libraries(${PROJECT_NAME} PUBLIC ${DEPEND_LIB} pybind11::module)
# target_link_libraries(kinematic_solver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/libkinematic_solver.so pybind11::module)

# # 设置安装路径
# set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/api/" CACHE PATH "Installation Directory" FORCE)

# install(DIRECTORY lib/ DESTINATION ./)

# install(TARGETS ${PROJECT_NAME}
#         LIBRARY DESTINATION ./${PROJECT_NAME}/
#         ARCHIVE DESTINATION ./${PROJECT_NAME}/)
# install(TARGETS kinematic_solver
#         LIBRARY DESTINATION ./${PROJECT_NAME}/
#         ARCHIVE DESTINATION ./${PROJECT_NAME}/)


set(DEPEND_LIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/libstartouch.so")



# 创建用户的可执行文件
# add_executable(user_app main.cpp)
# target_include_directories(user_app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib
#                     ${EIGEN3_INCLUDE_DIR}
#                                         )
# target_link_libraries(user_app PUBLIC ${DEPEND_LIB} ${PINOCCHIO_LIBRARIES} )

#[[
###生成.so文件  pybind.cpp 这个是带有皮诺曹和初始值的###
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/interface_py)
pybind11_add_module(startouch pybind.cpp)
# add_executable(controller_pybind examples/controller_pybind.cpp)
target_include_directories(startouch PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib
                    ${PINOCCHIO_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                                        )
target_link_libraries(startouch PRIVATE ${DEPEND_LIB} ${PINOCCHIO_LIBRARIES} )
]]


###生成.so文件
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/interface_py)
# pybind11_add_module(startouch pybind_wopin.cpp)  #这里是最后把参数辨识参数融进去的
pybind11_add_module(startouch pybind_0825testfast.cpp) #这个是需要额外的辨识参数

target_include_directories(startouch PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib
                    ${EIGEN3_INCLUDE_DIR}
                                        )
target_link_libraries(startouch PRIVATE ${DEPEND_LIB} Threads::Threads )