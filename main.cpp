//
//  main.cpp
//  MobileSensingSim
//
//  Created by Yuan on 4/15/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#include <iostream>
#include <fstream>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include "scenario_generator/scenario_generator.h"
#include "optimal_solver/graph_converter.h"
#include "solver_base.h"
#include "stat.h"
#include "optimal_solver/optimal_solver.h"
#include "optimal_solver/optimal_balance_solver.h"
#include "heuristic_solver/heuristic_solver.h"
#include "heuristic_solver/naive_solver.h"
#include "heuristic_solver/agg_heuristic_solver.h"
#include "heuristic_solver/heuristic_dyn_solver.h"

namespace {
  const char * DEFAULT_OUTFILE = "phonesim_result.txt";
}

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
  const int kScenarioNumber = 1;
  // Scenario parameters.
  mss::ScenarioParameters sp;
  sp.sensing_range = 40;
  sp.comm_range = 40;
  sp.running_time = 900;
  //	sp.phone_count = 40;
  sp.speed_range = mss::Range(5, 15, 0.1);
  sp.start_time_range = mss::Range(0, 200);
  sp.map = CreateMap();
  sp.data_per_second = 0.5;
  sp.sensing_cost_range = mss::Range(2, 6, 0.5);
  sp.transfer_cost_range = mss::Range(2, 6, 0.5);
  sp.upload_cost_range = mss::Range(2, 6, 0.5);
  sp.upload_limit_range = mss::Range(1, 3, 0.1);
  
  int phone_counts[] = {50};
  const int kPhoneCountsSize = 1;
  //  int phone_counts[] = {35};
  //  const int kPhoneCountsSize = 1;
  
  double dyn_muliples[] = {1.25};
  const int kDynMultipleSize = 1;
  
  std::ofstream of(DEFAULT_OUTFILE);
  for (int i = 0; i < kPhoneCountsSize; ++i) {
    std::vector<std::vector<mss::Statistics> > res_stats(4 + kDynMultipleSize,
                                                         std::vector<mss::Statistics>(3));
    for (int sid = 0; sid < kScenarioNumber; ++sid) {
      std::cout << "*****************************" << std::endl;
      std::cout << "Scenario ID: " << sid << std::endl;
      
      sp.phone_count = phone_counts[i];
      sp.seed = sid;
      
      // Create scneario generator.
      mss::ScenarioGenerator sg(sp);
      
      // Generate scenario.
      const mss::Scenario& scen = sg.GenerateDefaultScenario();
      
      // Create solvers
      std::vector<boost::shared_ptr<mss::SolverBase> > solvers;
      std::vector<std::string> solver_names;
      
      solvers.push_back(boost::shared_ptr<mss::SolverBase>(new mss::OptimalSolver()));
      solver_names.push_back("Optimal solver");
      
      solvers.push_back(boost::shared_ptr<mss::SolverBase>(new mss::OptimalBalanceSolver()));
      solver_names.push_back("Optimal balance solver");
      
      solvers.push_back(boost::shared_ptr<mss::SolverBase>(new mss::HeuristicSolver(60)));
      solver_names.push_back("Heuristic solver");
      
      //    mss::HeuristicSolver hbs(60, true);
      //    solvers.push_back(&hbs);
      //    solver_names.push_back("Heuristic balance solver");
      
      for (int j = 0; j < kDynMultipleSize; ++j) {
        solvers.push_back(boost::shared_ptr<mss::SolverBase>(new mss::HeuristicDynSolver(60, dyn_muliples[j])));
        std::string sname = "Heuristic Dynamic solver - Multiple = ";
        sname += boost::lexical_cast<std::string>(dyn_muliples[j]);
        solver_names.push_back(sname);
      }
      
      solvers.push_back(boost::shared_ptr<mss::SolverBase>(new mss::NaiveSolver()));
      solver_names.push_back("Naive solver");
      //mss::AggressiveHeuristicSolver ahs(60);
      //solvers.push_back(&ahs);
      //solver_names.push_back("Aggressive heuristic solver");
      
      assert(res_stats.size() == solvers.size());
      
      for (int j = 0; j < solvers.size(); ++j) {
        solvers[j]->SetMILP(false);
        std::cout << "Running algorithm " << solver_names[j] << std::endl;
        mss::Result r = solvers[j]->Solve(scen);
        
        // Save result to statistics if valid.
        if (r.is_valid && r.is_optimal) {
          //mss::Statistics phone_stat;
          std::vector<mss::Statistics> cost_stat(3);
          for (int k = 0; k < scen.phone_count; ++k) {
            //phone_stat.AddValue(r.PhoneCost(k));
            cost_stat[0].AddValue(r.phone_cost[k][mss::Cost::SENSING]);
            cost_stat[1].AddValue(r.phone_cost[k][mss::Cost::COMM]);
            cost_stat[2].AddValue(r.phone_cost[k][mss::Cost::UPLOAD]);
          }
//          res_stats[j][0].AddValue(r.all_cost);
//          res_stats[j][1].AddValue(r.MaxPhoneCost());
//          res_stats[j][2].AddValue(phone_stat.Variance());
          res_stats[j][0].AddValue(cost_stat[0].Mean());
          res_stats[j][1].AddValue(cost_stat[1].Mean());
          res_stats[j][2].AddValue(cost_stat[2].Mean());
        }
      }
    }
    
    // Write results.
    of << phone_counts[i] << "\t";
    for (int j = 0; j < res_stats.size(); ++j) {
      for (int k = 0; k < res_stats[j].size(); ++k)
      of << res_stats[j][k].Mean() << "\t";
    }
    of << std::endl;
  } //for int i
  
  return 0;
}

