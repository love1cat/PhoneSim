//
//  heuristic_solver.cpp
//  MobileSensingSim
//
//  Created by Yuan on 5/12/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#include "heuristic_solver.h"
#include "../simlog.h"

namespace mobile_sensing_sim {
  Result HeuristicSolver::Solve(const Scenario& scen) {
    hlog.Reset();
    hlog << "\n\n";
    hlog << "*********************************************\n";
    hlog << "Starting solve using heuristic algorithm\n";
    hlog << "*********************************************\n";
    Result r(scen.phone_count);
    
    GraphConverter gc;
    
    // Copy phones used in generating scenario.
    std::vector<Phone> phones = scen.phones;
    
    // Set phones to predict pattern only.
    // Try: Let phones only walk straight
    for (int i = 0; i < phones.size(); ++i) {
      TurnProbability tp;
      tp.straight = 1.0;
      tp.left = 0.0;
      tp.right = 0.0;
      phones[i].SetTurnProbability(tp);
    }
    
    // In online scenario, phones will not know that
    // new phones are coming, thus phone start vector
    // should be none.
    std::vector<std::vector<int> > start_phones(scen.scen_param.running_time, std::vector<int>());
    
    //**************************************************
    // Start simulated walk.
    // 1. Construct graph based on current locations.
    // 2. Adjust graph based on previous actions.
    //    a. Add edge at t if action happens at time t
    //    b. The capacity of the edge is how much data transferred at the moment.
    // 3. Solve converted graph by cplex solver.
    // 4. Execute actions returned by cplex solver.
    // 5. Phones move according to generated scenario.
    //**************************************************
    
    // Define data capacity vector to store phone's data transfers
    // (including sensing) over time.
    ThreeDimVector<double> phone_datatrans(scen.running_time, scen.phone_count, scen.phone_count + scen.target_count);
    phone_datatrans.Fill(0.0);
    
    // Start create and write scenario.
    hlog << "\n";
    hlog << "*********************************************\n";
    hlog << "Starting simulated walk\n";
    hlog << "*********************************************\n";
    
    std::vector<int> target_status(scen.target_count, 0);
    const int kDesiredTargetDataCount = 1;
    
    for (int t = 0; t < scen.running_time; t += report_period_) {
      // Enable phones if they start at current time or
      // since last report time (previous t).
      hlog << "\n";
      hlog << "*********************************************\n";
      hlog << "Time " << t << ":\n";
      hlog << "*********************************************\n";
      hlog << "Enable phones since last report time...\n";
      for (int i = t; i >= 0 && i > t - report_period_; --i) {
        for (int j = 0; j < scen.start_phones[i].size(); ++j) {
          int ph_id = scen.start_phones[i][j];
          hlog << "Enable phone " << ph_id << " at time " << i << ".\n";
          phones[ph_id].is_active_ = true;
        }
      }
      
      // Construct graph based on current locations
      // and empty phone start vector.
      ScenarioGenerator sg(scen.scen_param);
      Scenario cur_scen = sg.GenerateScenario(phones, start_phones, t);
      
      // Adjust adj mats and data mats based on
      // previous data sensings and transfers.
      hlog << "Adjust adjacency matrices and data capacity matrices based on previous data transfers...\n";
      for (int prevt = 0; prevt < t; ++prevt) {
        ThreeDimVector<double> &datatrans = phone_datatrans;
        for (int i = 0; i < scen.phone_count; ++i) {
          for (int j = 0; j < scen.phone_count + scen.target_count; ++j) {
            if (datatrans(prevt, i, j) != 0.0) {
              // If previous action is found
              // 1. Set corresponding edge in adjacency matrix.
              // 2. Set capacity of the edge to be transferred
              //    amount of data.
              hlog << "Previous data transfer / sensing found from vertex at time " << prevt << ": " << i << " to " << j << ", data amount: " << datatrans(prevt, i, j) << ".\n";
              ThreeDimVector<int> &ams = cur_scen.adj_mats;
              ams(prevt, i, j) = 1;
              ThreeDimVector<double> &dms = cur_scen.data_mats;
              dms(prevt, i, j) = datatrans(prevt, i, j);
            }
          }
        }
      }
      
      // Convert scenario to graph.
      hlog << "Converting adjusted scenario to graph...\n";
      gc.ConvertToGraph(cur_scen);
      
      // Solve converted graph.
      hlog << "Solve converted graph...\n";
      Solution cur_s;
      bool is_success = false;
      
      hlog << "Try optimal solver first...\n";
      CplexAdapter cplex_adapter;
      is_success = cplex_adapter.Solve(gc.GetGraph(), cur_s);
      
      if (!is_success) {
        ErrorHandler::RunningWarning("Cplex solver does not run successfully!");
        continue;
      }
      
      if (use_balance_) {
        Solution bal_s;
        cplex_balance_adapter_.SetMILP(false);
        hlog << "Optimal solver succeeded. Try optimal balanced solver...\n";
        is_success = cplex_balance_adapter_.Solve(gc.GetGraph(), cur_scen, BalanceOption(), bal_s);
        if (is_success) {
          cur_s = bal_s;
        }
      }
      
      if (UseMILP() && cur_s.solution_status == CPX_STAT_OPTIMAL) {
        // If feasible, try MILP
        Solution milp_s;
        is_success = false;
        if (use_balance_) {
          cplex_balance_adapter_.SetMILP(true);
          is_success = cplex_balance_adapter_.Solve(gc.GetGraph(), cur_scen, BalanceOption(), milp_s);
        } else {
          is_success = cplex_milp_adapter_.Solve(gc.GetGraph(), milp_s);
        }

        // If still feasible, use MILP solution.
        if (is_success) {
          cur_s = milp_s;
        }
      }
      
      r.is_valid = cur_s.is_valid;
      r.solution_status = cur_s.solution_status;
      r.is_optimal = (cur_s.solution_status == 1 || cur_s.solution_status == 101 || cur_s.solution_status == 102);
      
      // Execute actions returned by cplex solver.
      // Costs are added to final objective value.
      hlog << "Executing actions in the returned solution...\n";
      const std::vector<Edge>& edges = gc.GetEdges();
      assert(edges.size() == cur_s.edge_values.size());
      for (int i = 0; i < edges.size(); ++i) {
        const Edge& e = edges[i];
        if (cur_s.edge_values[i] == 0 || e.type == Edge::PHONE_TO_SELF || e.type == Edge::SRC_TO_TARGET) {
          continue;
        }
        if (e.time >= t && e.time < t + report_period_) {
          ThreeDimVector<double> &datatrans = phone_datatrans;
          double value = cur_s.edge_values[i];
          const ThreeDimVector<int> &ams = scen.adj_mats;
          if (e.type == Edge::TARGET_TO_PHONE) {
            assert(e.phone1_id != -1 && e.target_id != -1);
            // Make sure target is still in sensing range of the phone
            if (ams(e.time, e.phone1_id, e.target_id) != 0.0) {
              hlog << "Sensing action executed: phone " << e.phone1_id << " at target " << e.target_id << " at time " << e.time << ".\n";
              // Does not allow sensing part of the target.
              // Set data amount to be 1.0 all the time.
              //datatrans[e.phone1_id][e.target_id] = value;
              datatrans(e.time, e.phone1_id, e.target_id) = 1.0;
              double sensing_cost = scen.phones[e.phone1_id].costs_.sensing_cost * value;
              r.AddCost(e.phone1_id, sensing_cost, Cost::SENSING);
            } else {
              hlog << "Sensing action aborted: phone " << e.phone1_id << " is out of the range of target " << e.target_id << " at time " << e.time << ".\n";
            }
          } else if (e.type == Edge::PHONE_TO_PHONE) {
            assert(e.phone1_id != -1 && e.phone2_id != -1);
            // Make sure target is still in sensing range of the phone
            if (ams(e.time, e.phone1_id, e.phone2_id) != 0.0) {
              hlog << "Data transfer executed: phone " << e.phone1_id << " to phone " << e.phone2_id << ", data amount: " << cur_s.edge_values[i] << " at time " << e.time << ".\n";
              datatrans(e.time, e.phone1_id, e.phone2_id) = value;
              double comm_cost = (scen.phones[e.phone1_id].costs_.transfer_cost + scen.phones[e.phone2_id].costs_.transfer_cost) * value;
              r.AddCost(e.phone1_id, comm_cost, Cost::COMM);
            } else {
              hlog << "Data transfer aborted: phone " << e.phone1_id << " if out of the range of phone " << e.phone2_id << " at time " << e.time << ".\n";
            }
          } else if (e.type == Edge::PHONE_TO_SINK) {
            // Plus uploading cost at last iteration.
            if (t == scen.running_time - 1 || t + report_period_ >= scen.running_time) {
              assert(e.phone1_id != -1);
              hlog << "Uploading executed: phone " << e.phone1_id << ", data amount: " << value << ".\n";
              double upload_cost = scen.phones[e.phone1_id].costs_.upload_cost * value;
              r.AddCost(e.phone1_id, upload_cost, Cost::UPLOAD);
            }
          } else {
            ErrorHandler::RunningError("Heuristic algorithm: Unkown edge type is found while executing actions returned by cplex solver!");
          }
        }
      }
      
      //			// Don't miss sensing oppurtunities.
      //			// If target is in range and not sensed by any phone,
      //			// sense it.
      //
      //			// Update target status.
      //			// Some phones carrying data may be out of the region.
      //			for (int i = 0; i < phone_count)
      //
      //			for (int i = t; i < t + report_period_; ++i) {
      //				const AdjacencyMatrix &am = scen.adj_mats[i];
      //			}
      //
      //			for (int i = t; i < t + report_period_; ++i) {
      //
      //			}
      
      // Phones move according to generated scenario.
      hlog << "Moving phones according to generated scneario...\n";
      if (t + report_period_ < scen.running_time) {
        for (int i = 0; i < phones.size(); ++i) {
          if (phones[i].is_active_) {
            phones[i].MoveTo(scen.phone_locations[t + report_period_][i]);
          }
        }
      }
      
      // If last iteration, record solution found.
      if (t == scen.running_time - 1 || t + report_period_ >= scen.running_time) {
        r.is_valid = cur_s.is_valid;
        r.solution_status = cur_s.solution_status;
        r.is_optimal = (cur_s.solution_status == 1 || cur_s.solution_status == 101 || cur_s.solution_status == 102);
        
        hlog << "\n";
        hlog << "*********************************************\n";
        hlog << "Final solution: \n";
        hlog << "*********************************************\n";
        if (!cur_s.is_valid) {
          hlog << "Not found a valid solution!\n";
          break;
        }
        
        assert(cur_s.edge_count == gc.GetGraph().edge_count);
        hlog << "Objective value: " << r.all_cost << "\n";
        hlog << "Objective status: " << cur_s.solution_status << "\n";
        for (int i = 0; i < cur_s.edge_values.size(); ++i) {
          if (cur_s.edge_values[i] != 0.0 && gc.GetEdge(i).type != Edge::PHONE_TO_SELF) {
            hlog << "name: " << gc.GetEdge(i).name << "\t value: " << cur_s.edge_values[i] << "\n";
          }
        }
      }
    }
    
    return r;
  }
}
