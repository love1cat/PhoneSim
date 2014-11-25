cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

SET(AppName phonesim)

project(MobilePhoneSimProject)

# FIND_PACKAGE(boost)

INCLUDE_DIRECTORIES("~/Tools/IBM/ILOG/CPLEX_Studio1251/cplex/include")
INCLUDE_DIRECTORIES("~/Tools/IBM/ILOG/CPLEX_Studio1251/concert/include") 
INCLUDE_DIRECTORIES("~/Library/boost_1_55_0") 

# ADD_SUBDIRECTORY(heuristic_solver) 
# ADD_SUBDIRECTORY(optimal_solver) 
# ADD_SUBDIRECTORY(scenario_generator) 

link_directories("~/Tools/IBM/ILOG/CPLEX_Studio1251/cplex/lib/x86-64_sles10_4.1/static_pic/")
link_directories("~/Tools/IBM/ILOG/CPLEX_Studio1251/concert/lib/x86-64_sles10_4.1/static_pic/")

add_definitions(-DIL_STD)

add_executable(${AppName} error_handler.h main.cpp simlog.h simlog.cpp
  milp_base.h solver_base.h stat.h
    optimal_solver/cplex_adapter_base.h optimal_solver/cplex_adapter_base.cpp
    optimal_solver/cplex_adapter.h optimal_solver/cplex_adapter.cpp 
    optimal_solver/cplex_milp_adapter.h optimal_solver/cplex_milp_adapter.cpp 
    optimal_solver/cplex_balance_adapter.h optimal_solver/cplex_balance_adapter.cpp
    optimal_solver/graph_converter.h optimal_solver/graph_converter.cpp
    optimal_solver/optimal_solver.h optimal_solver/optimal_solver.cpp
    optimal_solver/optimal_balance_solver.h optimal_solver/optimal_balance_solver.cpp
    heuristic_solver/heuristic_solver.h heuristic_solver/heuristic_solver.cpp 
    heuristic_solver/naive_solver.h heuristic_solver/naive_solver.cpp 
    heuristic_solver/agg_heuristic_solver.h heuristic_solver/agg_heuristic_solver.cpp
    heuristic_solver/heuristic_dyn_solver.h heuristic_solver/heuristic_dyn_solver.cpp
    scenario_generator/area_map.h scenario_generator/monitor_map.h scenario_generator/multidim_vector.h scenario_generator/phone.h scenario_generator/phone.cpp
    scenario_generator/random_generator.cpp scenario_generator/random_generator.h scenario_generator/scenario_generator.h
    scenario_generator/scenario_generator.cpp)

target_link_libraries(${AppName} ilocplex concert cplex m pthread)
      
get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
    message(STATUS "dir='${dir}'")
endforeach()
