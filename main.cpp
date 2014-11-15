//
//  main.cpp
//  MobileSensingSim
//
//  Created by Yuan on 4/15/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#include <iostream>
#include <fstream>
#include "scenario_generator/scenario_generator.h"
#include "optimal_solver/graph_converter.h"
#include "solver_base.h"
#include "optimal_solver/optimal_solver.h"
#include "optimal_solver/optimal_balance_solver.h"
#include "heuristic_solver/heuristic_solver.h"
#include "heuristic_solver/naive_solver.h"
#include "heuristic_solver/agg_heuristic_solver.h"

namespace mss = mobile_sensing_sim;

mss::MonitorMap CreateMap() {
  // Length and width.
  int length = 624;
  int width = 316;
  
  // Entry points.
  std::vector<mss::Point> entry_points;
  entry_points.push_back(mss::Point(156, 0));
  entry_points.push_back(mss::Point(312, 0));
  entry_points.push_back(mss::Point(468, 0));
  entry_points.push_back(mss::Point(0, 79));
  entry_points.push_back(mss::Point(0, 158));
  entry_points.push_back(mss::Point(0, 237));
  entry_points.push_back(mss::Point(156, 316));
  entry_points.push_back(mss::Point(312, 316));
  entry_points.push_back(mss::Point(468, 316));
  entry_points.push_back(mss::Point(624, 79));
  entry_points.push_back(mss::Point(624, 158));
  entry_points.push_back(mss::Point(624, 237));
  
  // Intersect points.
  std::vector<mss::Point> intersect_points;
  intersect_points.push_back(mss::Point(156, 79));
  intersect_points.push_back(mss::Point(156, 158));
  intersect_points.push_back(mss::Point(156, 237));
  intersect_points.push_back(mss::Point(468, 79));
  intersect_points.push_back(mss::Point(468, 158));
  intersect_points.push_back(mss::Point(468, 237));
  intersect_points.push_back(mss::Point(312, 79));
  intersect_points.push_back(mss::Point(312, 158));
  intersect_points.push_back(mss::Point(312, 237));
  
  // Create area map.
  mss::AreaMap am(entry_points, intersect_points, length, width);
  
  // Monitor / target points.
  std::vector<mss::Point> monitor_points;
  monitor_points.push_back(mss::Point(156, 79));
  monitor_points.push_back(mss::Point(156, 237));
  monitor_points.push_back(mss::Point(468, 79));
  monitor_points.push_back(mss::Point(468, 237));
  monitor_points.push_back(mss::Point(312, 158));
  // Create monitor map.
  return mss::MonitorMap(monitor_points, am);
}



int main(int argc, const char * argv[])
{
  // Scenario parameters.
  mss::ScenarioParameters sp;
  sp.sensing_range = 40;
  sp.comm_range = 40;
  sp.running_time = 900;
  //	sp.phone_count = 40;
  sp.speed_range = mss::Range(5, 15, 0.1);
  sp.start_time_range = mss::Range(0, 200);
  sp.seed = 0;
  sp.map = CreateMap();
  sp.data_per_second = 0.5;
  sp.sensing_cost_range = mss::Range(2, 6, 0.5);
  sp.transfer_cost_range = mss::Range(2, 6, 0.5);
  sp.upload_cost_range = mss::Range(2, 6, 0.5);
  sp.upload_limit_range = mss::Range(1, 5, 0.1);
  
  //  int phone_counts[] = {35, 40, 45, 50};
  //  const int kPhoneCountsSize = 4;
  int phone_counts[] = {35};
  const int kPhoneCountsSize = 1;
  std::ofstream outfile("diff_phone_counts.txt");
  std::ofstream sensingcost_file("sensing_costs.txt");
  std::ofstream commcost_file("comm_costs.txt");
  std::ofstream uploadcost_file("upload_costs.txt");
  std::ofstream statusfile("algorithm_status.txt");
  for (int i = 0; i < kPhoneCountsSize; ++i) {
    sp.phone_count = phone_counts[i];
    outfile << phone_counts[i] << "\t";
    sensingcost_file << phone_counts[i] << "\t";
    commcost_file << phone_counts[i] << "\t";
    uploadcost_file << phone_counts[i] << "\t";
    statusfile << phone_counts[i] << "\t";
    
    // Create scneario generator.
    mss::ScenarioGenerator sg(sp);
    
    // Generate scenario.
    const mss::Scenario& scen = sg.GenerateDefaultScenario();
    
    // Create solvers
    std::vector<mss::SolverBase *> solvers;
    std::vector<std::string> solver_names;
    //		mss::OptimalSolver os;
    //		solvers.push_back(&os);
    //		solver_names.push_back("Optimal solver");
    //		mss::HeuristicSolver hs(60);
    //		solvers.push_back(&hs);
    //		solver_names.push_back("Heuristic solver");
    //		mss::NaiveSolver ns;
    //		solvers.push_back(&ns);
    //		solver_names.push_back("Naive solver");
    //    mss::AggressiveHeuristicSolver ahs(60);
    //    solvers.push_back(&ahs);
    //    solver_names.push_back("Aggressive heuristic solver");
    mss::OptimalBalanceSolver obs;
    solvers.push_back(&obs);
    solver_names.push_back("Optimal balance solver");
    for (int j = 0; j < solvers.size(); ++j) {
      mss::Solution s = solvers[j]->Solve(scen);
      outfile << s.obj << "\t";
      sensingcost_file << s.sensing_cost << "\t";
      commcost_file << s.comm_cost << "\t";
      uploadcost_file << s.upload_cost << "\t";
      statusfile << s.solution_status << "\t";
    }
    
    outfile << "\n";
    sensingcost_file << "\n";
    commcost_file << "\n";
    uploadcost_file << "\n";
    statusfile << "\n";
  } //for int i
  
  return 0;
}

