cmake_minimum_required(VERSION 3.0.2)
#cmake_minimum_required(VERSION 3.7)
project(model_predictive_contouring_control)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
)

find_package(PythonLibs REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(PythonLibs 2.7)
include_directories(${PYTHON_INCLUDE_DIRS})


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O2")

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    std_msgs
    roscpp
    
)


include_directories(.)
include_directories(External/blasfeo/lib/include)
include_directories(External/hpipm/lib/include)
include_directories(External/matplotlib)
include_directories(External/Eigen)
include_directories(External/Json/include)

add_executable(MPCC
        main.cpp
        types.cpp
        types.h
        config.h
        Params/params.cpp
        Spline/cubic_spline.cpp
        Spline/arc_length_spline.cpp
        Interfaces/hpipm_interface.cpp
        Interfaces/solver_interface.cpp
        Constraints/constraints.cpp
        Constraints/bounds.cpp
        Cost/cost.cpp
        Model/bicycle_model.cpp
        Model/integrator.cpp
        Tests/spline_test.cpp
        Tests/cost_test.cpp
        Tests/model_integrator_test.cpp
        Tests/constratins_test.cpp
        MPC/mpc.cpp
        Params/track.cpp
        Params/track.h
        Plotting/plotting.cpp
        Plotting/plotting.h
        )

#find_package(Python COMPONENTS Development)
#target_include_directories(MPCC PRIVATE ${Python_INCLUDE_DIRS})

target_link_libraries(MPCC /home/syscon/mpc_ws/src/cpp_mpcc/C++/External/hpipm/lib/lib/libhpipm.a /home/syscon/mpc_ws/src/cpp_mpcc/C++/External/blasfeo/lib/lib/libblasfeo.a ${catkin_LIBRARIES} ${Python_LIBRARIES} )
#target_link_libraries(MPCC ${Python_LIBRARIES})
#target_link_libraries(MPCC ${Python_LIBRARIES} PYTHON_INCLUDE_DIRS)