//
//  optimal_balance_solver.cpp
//  PhoneSim
//
//  Created by Yuan on 11/11/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#include "optimal_balance_solver.h"
#include "graph_converter.h"
#include "../simlog.h"

namespace mobile_sensing_sim {
  Solution OptimalBalanceSolver::Solve(const Scenario& scen) {
    oblog.Reset();
    gc_.ConvertToGraph(scen);
    const Graph& g = gc_.GetGraph();
    //		gc.PrintInformation();
    Solution s;
    
    BalanceOption bo;
    
    cplex_adapter_.Solve(g, scen, bo, s);
    
//    // Recompute objective value as we may have used
//    // time-related sensing costs.
//    s.obj = 0.0;
//    s.sensing_cost = 0.0;
//    s.comm_cost = 0.0;
//    s.upload_cost = 0.0;
//    for (int i = 0; i < s.edge_count; ++i) {
//      if (s.edge_values[i] != 0.0) {
//        const Edge& e = gc_.GetEdge(i);
//        if (e.type == Edge::SRC_TO_TARGET || e.type == Edge::PHONE_TO_SELF) {
//          continue;
//        }
//        double value = s.edge_values[i];
//        if (e.type == Edge::TARGET_TO_PHONE) {
//          double sensing_cost = scen.phones[e.phone1_id].costs_.sensing_cost * value;
//          s.sensing_cost += sensing_cost;
//          s.obj += sensing_cost;
//        } else if (e.type == Edge::PHONE_TO_PHONE) {
//          double comm_cost = (scen.phones[e.phone1_id].costs_.transfer_cost + scen.phones[e.phone2_id].costs_.transfer_cost) * value;
//          s.comm_cost += comm_cost;
//          s.obj += comm_cost;
//        } else if (e.type == Edge::PHONE_TO_SINK) {
//          double upload_cost = scen.phones[e.phone1_id].costs_.upload_cost * value;
//          s.upload_cost += upload_cost;
//          s.obj += upload_cost;
//        } else {
//          ErrorHandler::RunningError("Optimal algorithm: Unkown edge type is found while adjust solution returned by cplex solver!");
//        }
//      }
//    }
//    
    oblog << "\n";
    oblog << "*********************************************\n";
    oblog << "Final solution: \n";
    oblog << "*********************************************\n";
    if (!s.is_valid) {
      oblog << "Not found a valid solution!\n";
      return s;
    }
    
    assert(s.edge_count == gc_.GetGraph().edge_count);
    oblog << "Objective value: " << s.obj << "\n";
    oblog << "Objective status: " << s.solution_status << "\n";
    for (int i = 0; i < s.edge_values.size(); ++i) {
      if (s.edge_values[i] != 0.0 && gc_.GetEdge(i).type != Edge::PHONE_TO_SELF) {
        oblog << "name: " << gc_.GetEdge(i).name << "\t value: " << s.edge_values[i] << "\n";
      }
    }
    
    return s;
  }
}
