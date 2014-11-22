//
//  agg_heuristic_solver.cpp
//  PhoneSim
//
//  Created by Yuan on 7/11/14.
//

#include "agg_heuristic_solver.h"
#include "../simlog.h"

namespace mobile_sensing_sim {
  Result AggressiveHeuristicSolver::Solve(const mobile_sensing_sim::Scenario &scen) {
    ahlog.Reset();
    ahlog << "\n\n";
    ahlog << "*********************************************\n";
    ahlog << "Starting solve using aggresive heuristic algorithm\n";
    ahlog << "*********************************************\n";
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
    ahlog << "\n";
    ahlog << "*********************************************\n";
    ahlog << "Starting simulated walk\n";
    ahlog << "*********************************************\n";
    
    // 07/31/2014
    // e.target_seqid ==> 0 - target_count - 1
    // e.target_id = phone_count + e.target_seqid
    // ams use e.target_id
    
    std::vector<int> target_status(scen.target_count, 0);
    const int kDesiredTargetDataCount = 1;
    
    // Create a vector to track data sensed by phones.
    std::vector<std::vector<double> > data_received;
    for (int i = 0; i < scen.phone_count; ++i) {
      data_received.push_back(std::vector<double>(scen.target_count, 0.0));
    }
    
    // Create a vector to track how much data is uploaded for each target.
    std::vector<double> target_uploaded(scen.target_count, 0.0);
    
    double previous_upload_amount = 0.0;
    for (int t = 0; t < scen.running_time; t += report_period_) {
      double current_upload_amount = 0.0;
      // Enable phones if they start at current time or
      // since last report time (previous t).
      ahlog << "\n";
      ahlog << "*********************************************\n";
      ahlog << "Time " << t << ":\n";
      ahlog << "*********************************************\n";
      ahlog << "Enable phones since last report time...\n";
      for (int i = t; i >= 0 && i > t - report_period_; --i) {
        for (int j = 0; j < scen.start_phones[i].size(); ++j) {
          int ph_id = scen.start_phones[i][j];
          ahlog << "Enable phone " << ph_id << " at time " << i << ".\n";
          phones[ph_id].is_active_ = true;
        }
      }
      
      // Construct graph based on current locations
      // and empty phone start vector.
      ScenarioGenerator sg(scen.scen_param);
      Scenario cur_scen = sg.GenerateScenario(phones, start_phones, t);
      
      // Adjust adj mats and data mats based on
      // previous data sensings and transfers.
      ahlog << "Adjust adjacency matrices and data capacity matrices based on previous data transfers...\n";
      for (int prevt = 0; prevt < t; ++prevt) {
        ThreeDimVector<double> &datatrans = phone_datatrans;
        for (int i = 0; i < scen.phone_count; ++i) {
          for (int j = 0; j < scen.phone_count + scen.target_count; ++j) {
            if (datatrans(prevt, i, j) != 0.0) {
              // If previous action is found
              // 1. Set corresponding edge in adjacency matrix.
              // 2. Set capacity of the edge to be transferred
              //    amount of data.
              ahlog << "Previous data transfer / sensing found from vertex at time " << prevt << ": " << i << " to " << j << ", data amount: " << datatrans(prevt, i, j) << ".\n";
              ThreeDimVector<int> &ams = cur_scen.adj_mats;
              ams(prevt, i, j) = 1;
              ThreeDimVector<double> &dms = cur_scen.data_mats;
              dms(prevt, i, j) = datatrans(prevt, i, j);
            }
          }
        }
      }
      
      // Adjust scen running time to current time so that phone may
      // upload data earlier. (upload at t + report_period_ - 1)
      int org_running_time = cur_scen.running_time;
      cur_scen.running_time = t + report_period_;
      
      // Convert scenario to graph.
      ahlog << "Converting adjusted scenario to graph...\n";
      gc.ConvertToGraph(cur_scen);
      
      // Solve converted graph.
      ahlog << "Solve converted graph...\n";
      Solution cur_s;
      bool solution_status = cplex_adapter_.Solve(gc.GetGraph(), cur_s);
      if (!solution_status) {
        ErrorHandler::RunningError("Cplex solver does not run successfully!");
      }
      
      if (UseMILP() && cur_s.solution_status == CPX_STAT_OPTIMAL) {
        // If feasible, try MILP
        Solution milp_s;
        bool status = cplex_milp_adapter_.Solve(gc.GetGraph(), milp_s);
        // If still feasible, use MILP solution.
        if (status) {
          cur_s = milp_s;
        }
      }
      
      r.is_valid = cur_s.is_valid;
      r.solution_status = cur_s.solution_status;
      r.is_optimal = (cur_s.solution_status == 1 || cur_s.solution_status == 101 || cur_s.solution_status == 102);
      
      // Execute actions returned by cplex solver.
      // Costs are added to final objective value.
      ahlog << "Executing actions in the returned solution...\n";
      const std::vector<Edge>& edges = gc.GetEdges();
      assert(edges.size() == cur_s.edge_values.size());
      for (int i = 0; i < edges.size(); ++i) {
        const Edge& e = edges[i];
        if (cur_s.edge_values[i] == 0 || e.type == Edge::PHONE_TO_SELF) {
          continue;
        }
        if (e.type == Edge::SRC_TO_TARGET) {
          // Indicator of how much data has been uploaded.
          target_uploaded[e.target_seqid] = cur_s.edge_values[i];
          current_upload_amount += cur_s.edge_values[i];
          continue;
        }
        
        if (e.time >= t && e.time < t + report_period_) {
          ThreeDimVector<double> &datatrans = phone_datatrans;
          double value = cur_s.edge_values[i];
          const ThreeDimVector<int> &ams = scen.adj_mats;
          if (e.type == Edge::TARGET_TO_PHONE) {
            assert(e.phone1_id != -1 && e.target_seqid != -1);
            
            // Make sure target is still in sensing range of the phone
            if (target_uploaded[e.target_seqid] != 1.0 && data_received[e.phone1_id][e.target_seqid] != 1.0 && ams(e.time, e.phone1_id, e.target_id) != 0.0) {
              ahlog << "Sensing action executed: phone " << e.phone1_id << " at target " << e.target_id << " at time " << e.time << ".\n";
              // Does not allow sensing part of the target.
              // Set data amount to be 1.0 all the time.
              //datatrans[e.phone1_id][e.target_id] = value;
              datatrans(e.time, e.phone1_id, e.target_id) = 1.0;
              double sensing_cost = scen.phones[e.phone1_id].costs_.sensing_cost * value;
              r.AddCost(e.phone1_id, sensing_cost, Cost::SENSING);
              
              // Record data received.
              data_received[e.phone1_id][e.target_seqid] = 1.0;
              
            } else {
              ahlog << "Sensing action aborted: phone " << e.phone1_id << " is out of the range of target or the target is fully uploaded." << e.target_id << " at time " << e.time << ".\n";
            }
          } else if (e.type == Edge::PHONE_TO_PHONE) {
            assert(e.phone1_id != -1 && e.phone2_id != -1);
            // Make sure two phones are still in communication range
            // of each other.
            if (ams(e.time, e.phone1_id, e.phone2_id) != 0.0) {
              ahlog << "Data transfer executed: phone " << e.phone1_id << " to phone " << e.phone2_id << ", data amount: " << cur_s.edge_values[i] << " at time " << e.time << ".\n";
              datatrans(e.time, e.phone1_id, e.phone2_id) = value;
              double comm_cost = (scen.phones[e.phone1_id].costs_.transfer_cost + scen.phones[e.phone2_id].costs_.transfer_cost) * value;
              r.AddCost(e.phone1_id, comm_cost, Cost::COMM);
            } else {
              ahlog << "Data transfer aborted: phone " << e.phone1_id << " if out of the range of phone " << e.phone2_id << " at time " << e.time << ".\n";
            }
          } else {
            ErrorHandler::RunningError("Heuristic algorithm: Unkown edge type is found while executing actions returned by cplex solver!");
          }
        } // if (e.time >= t && e.time < t + report_period_)
      } //  for i
      
      // Compute uploading cost seperately.
      double diff = current_upload_amount - previous_upload_amount;
      if (diff > 0.0) {
        for (int i = 0; i < edges.size(); ++i) {
          const Edge& e = edges[i];
          if (cur_s.edge_values[i] == 0 || e.type == Edge::PHONE_TO_SELF) {
            continue;
          }
          if (e.time >= t && e.time < t + report_period_) {
            ThreeDimVector<double> &datatrans = phone_datatrans;
            double value = cur_s.edge_values[i];
            const ThreeDimVector<int> &ams = scen.adj_mats;
            if (e.type == Edge::PHONE_TO_SINK) {
              // Plus uploading cost.
              
              assert(e.phone1_id != -1);
              ahlog << "Uploading executed: phone " << e.phone1_id << ", data amount: " << value << ".\n";
              
              // Only count in recent uploads.
              if (diff > 0) {
                if (diff < value) {
                  value  = diff;
                }
                double upload_cost = scen.phones[e.phone1_id].costs_.upload_cost * value;
                r.AddCost(e.phone1_id, upload_cost, Cost::UPLOAD);
                diff -= value;
              } else {
                break;
              }
            }
          }
        }
      }
      previous_upload_amount = current_upload_amount;
      
      // Stop if all targets are uploaded.
      bool is_all_uploaded = true;
      for (int i = 0; i < scen.target_count; ++i) {
        if (target_uploaded[i] != 1.0) {
          is_all_uploaded = false;
          break;
        }
      }
      
      if (is_all_uploaded) {
        r.solution_status = cur_s.solution_status;
        r.is_valid = cur_s.is_valid;
        r.is_optimal = (cur_s.solution_status == 1 || cur_s.solution_status == 101 || cur_s.solution_status == 102);
        
        // Record solution and return.
        ahlog << "\n";
        ahlog << "*********************************************\n";
        ahlog << "Final solution: \n";
        ahlog << "*********************************************\n";
        if (!cur_s.is_valid) {
          ahlog << "Not found a valid solution!\n";
          break;
        }
        
        assert(cur_s.edge_count == gc.GetGraph().edge_count);
        ahlog << "Objective value: " << r.all_cost << "\n";
        ahlog << "Objective status: " << cur_s.solution_status << "\n";
        for (int i = 0; i < cur_s.edge_values.size(); ++i) {
          if (cur_s.edge_values[i] != 0.0 && gc.GetEdge(i).type != Edge::PHONE_TO_SELF) {
            ahlog << "name: " << gc.GetEdge(i).name << "\t value: " << cur_s.edge_values[i] << "\n";
          }
        }
        return r;
      }
      
      // Don't miss sensing oppurtunities.
      // If target is in range and not fully uploaded and
      // not sensed before, sense it.
      
      for (int i = 0; i < scen.phone_count; ++i) {
        for (int j = 0; j < scen.target_count; ++j) {
          if (target_uploaded[j] == 1 || data_received[i][j] == 1) {
            continue;
          }
          int target_id = j + scen.phone_count;
          for (int k = t; k < t + report_period_; ++k) {
            const ThreeDimVector<int> &ams = scen.adj_mats;
            if (data_received[i][j] != 1.0 && ams(k, i, target_id) != 0.0 ) {
              ThreeDimVector<double> &datatrans = phone_datatrans;
              datatrans(k, i, j) = 1.0;
              double sensing_cost = scen.phones[i].costs_.sensing_cost;
              r.AddCost(i, sensing_cost, Cost::SENSING);
              
              // Record data received.
              data_received[i][j] = 1.0;
            }
          }
        }
      }
      
      // Phones move according to generated scenario.
      ahlog << "Moving phones according to generated scneario...\n";
      if (t + report_period_ < scen.running_time) {
        for (int i = 0; i < phones.size(); ++i) {
          if (phones[i].is_active_) {
            phones[i].MoveTo(scen.phone_locations[t + report_period_][i]);
          }
        }
      }
      
      // If last iteration, record solution found.
      if (t == scen.running_time - 1 || t + report_period_ >= scen.running_time) {
        r.solution_status = cur_s.solution_status;
        r.is_valid = cur_s.is_valid;
        r.is_optimal = (cur_s.solution_status == 1 || cur_s.solution_status == 101 || cur_s.solution_status == 102);
        
        ahlog << "\n";
        ahlog << "*********************************************\n";
        ahlog << "Final solution: \n";
        ahlog << "*********************************************\n";

        if (!cur_s.is_valid) {
          ahlog << "Not found a valid solution!\n";
          break;
        }
        
        assert(cur_s.edge_count == gc.GetGraph().edge_count);
        ahlog << "Objective value: " << r.all_cost << "\n";
        ahlog << "Objective status: " << cur_s.solution_status << "\n";
        for (int i = 0; i < cur_s.edge_values.size(); ++i) {
          if (cur_s.edge_values[i] != 0.0 && gc.GetEdge(i).type != Edge::PHONE_TO_SELF) {
            ahlog << "name: " << gc.GetEdge(i).name << "\t value: " << cur_s.edge_values[i] << "\n";
          }
        }
      }
    } // for t
    
    return r;
  }
}
