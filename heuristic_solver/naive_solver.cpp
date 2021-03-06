//
//  naive_solver.cpp
//  MobileSensingSim
//
//  Created by Yuan on 5/19/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#include "naive_solver.h"
#include "../simlog.h"

namespace mobile_sensing_sim {
  Result NaiveSolver::Solve(const Scenario &scen) {
    if (UseMILP()) {
      return SolveMILP(scen);
    }
    
    return SolveNonMILP(scen);
  }
  
  Result NaiveSolver::SolveMILP(const Scenario &scen) {
    nlog.Reset();
    Result r(scen.phone_count);
    
    // Create global required data info.
    // (i, j): celluar tower's knowledge on how much more data
    // of target j sensed by phone i has yet to be uploaded.
    // Initialized with 1.0: all need to be uploaded.
    DataInfo data_required;
    for (int i = 0; i < scen.phone_count; ++i) {
      for (int j = 0; j < scen.target_count; ++j) {
        data_required[std::make_pair(i, j)] = 1.0;
      }
    }
    
    // Create target status.
    // False: not uploaded yet. True: uploaded.
    std::vector<bool> target_status(scen.target_count, false);
    
    // Create phone data matrix
    // Each phone has a data info matrix indicating
    // how much sensing data (associated with phones) it currently has.
    std::vector<DataInfo> phone_datas(scen.phone_count);
    
    // Copy phones upload limits.
    std::vector<double> upload_limits(scen.phone_count, 0.0);
    for (int i = 0; i < scen.phone_count; ++i) {
      upload_limits[i] = scen.phones[i].upload_limit_;
    }
    
    // Simulate walk.
    nlog << "\n";
    nlog << "*********************************************\n";
    nlog << "Start simulated walk: \n";
    nlog << "*********************************************\n";
    
    for (int t = 0; t < scen.running_time; ++t) {
      nlog << "\n";
      nlog << "*********************************************\n";
      nlog << "Time: " << t << ":\n";
      const ThreeDimVector<int> &am = scen.adj_mats;
      const ThreeDimVector<double> &dm = scen.data_mats;
      for (int i = 0; i < scen.phone_count; ++i) {
        DataInfo &di = phone_datas[i];
        
        // Update data storage as all or part of targets' data
        // may have been uploaded.
        //nlog << "Phone " << i << " update data storage...\n";
        if (!di.empty()) {
          for (DataInfo::iterator it = di.begin(); it != di.end();) {
            int tid = (*it).first.second;
            if (target_status[tid] == true) {
              di.erase(it++);
            } else {
              double data = (*it).second;
              std::pair<int, int> phone_target = (*it).first;
              if (data > data_required[phone_target]) {
                (*it).second = data_required[phone_target];
              }
              ++it;
            }
          }
        }
        
        // Check if we can sense any target.
        for (int j = 0; j < scen.target_count; ++j) {
          // Target j's id in adj mat is phone count + j
          if (am(t, i, scen.phone_count + j) == 1 && target_status[j] == false) {
            DataInfo::iterator find_it = di.find(std::make_pair(i, j));
            if (find_it == di.end()) {
              // Target j is not fully uploaded and is not
              // in data storage.
              // Sense the target and save the data.
              nlog << "Phone " << i << " sense target " << j << ".\n";
              double sensing_cost = scen.phones[i].costs_.sensing_cost;
              r.AddCost(i, sensing_cost, Cost::SENSING);
              di[std::make_pair(i, j)] = data_required[std::make_pair(i, j)];
            }
          }
        }
        
        // Check if we have data that can be transferred.
        if (!di.empty()) {
          // There is data can be transferred.
          // Check if we can upload the data we have.
          for (DataInfo::iterator it = di.begin(); upload_limits[i] > 0 && it != di.end();) {
            // Data amount is (*it).second
            if ((*it).second > upload_limits[i]) {
              // Only be able to upload part of the data.
              nlog << "Phone " << i << " uploads part of target " << (*it).first.second << ", upload amount: " << upload_limits[i] << ".\n";
              (*it).second -= upload_limits[i];
              double upload_cost = scen.phones[i].costs_.upload_cost * upload_limits[i];
              r.AddCost(i, upload_cost, Cost::UPLOAD);
              
              // Update global required data info.
              if ((*it).second < data_required[(*it).first]) {
                data_required[(*it).first] = (*it).second;
              }
              
              // Upload limit is used up.
              upload_limits[i] = 0.0;
              break;
            } else {
              // Can upload all the data.
              // The corresponding target data is uploaded.
              nlog << "Phone " << i << " uploads all of (" << (*it).first.first << "," << (*it).first.second << "), upload amount: " << (*it).second << ".\n";
              int tid = (*it).first.second;
              target_status[tid] = true;
              // Remove this data from storage.
              upload_limits[i] -= (*it).second;
              double upload_cost = scen.phones[i].costs_.upload_cost * (*it).second;
              r.AddCost(i, upload_cost, Cost::UPLOAD);
              di.erase(it++);
            }
          }
          
          // We may have uploaded all the data.
          if (di.empty()) {
            continue;
          }
          
          // Check if there is an available neighbor.
          double amount_transferred = 0.0;
          for (int j = 0; j < scen.phone_count; ++j) {
            if (i != j && am(t, i, j) == 1) {
              for (DataInfo::iterator it = di.begin(); it != di.end() && amount_transferred < dm(t, i, j); ++it) {
                // Search on phone j to see if phone j already
                // has the data. Only transfer the data phone j
                // does not have.
                DataInfo::iterator find_it = phone_datas[j].find((*it).first);
                if (find_it == phone_datas[j].end()) {
                  // Assume all the data for current target can be
                  // transferred. (all dm[i][j] >= 1.0)
                  nlog << "Phone " << i << " copy all of (" << (*it).first.first << "," << (*it).first.second << ") to phone " << j << ", transfer amount: " << data_required[(*it).first] << ".\n";
                  double data_transferred = data_required[(*it).first];
                  double comm_cost1 = scen.phones[i].costs_.transfer_cost * data_transferred;
                  double comm_cost2 = scen.phones[j].costs_.transfer_cost * data_transferred;
                  phone_datas[j][(*it).first] = data_transferred;
                  amount_transferred += data_transferred;
                  r.AddCost(i, comm_cost1, Cost::COMM);
                  r.AddCost(j, comm_cost2, Cost::COMM);
                }
              }
              // break; // allow only one transfer.
            }
          } // End of for (int j
        } // End of if (!di.empty())
      } // End of for (int i
      
      // Check if all target datas are uploaded
      bool all_upload = true;
      for (int i = 0; i < scen.target_count; ++i) {
        if (target_status[i] == false) {
          all_upload = false;
          break;
        }
      }
      if (all_upload == true) {
        break;
        nlog << "All set. Exit loop.\n";
      }
      
    } // End of for (int t
    
    // Check if all target datas are uploaded
    bool all_upload = true;
    for (int i = 0; i < scen.target_count; ++i) {
      if (target_status[i] == false) {
        all_upload = false;
        break;
      }
    }
    if (!all_upload) {
      r.is_valid = false; // infeasible
      r.is_optimal = false;
    } else {
      r.is_valid = true; // valid
      r.is_optimal = true;
    }
    
    return r;
  }
  
  Result NaiveSolver::SolveNonMILP(const Scenario &scen) {
    nlog.Reset();
    Result r(scen.phone_count);
    
    // Create global required data info.
    typedef std::vector<std::vector<double> > DataInfo_t;
    DataInfo_t phone_datas(scen.phone_count, std::vector<double>(scen.target_count, 0.0));
    
    // Create target status.
    // False: not uploaded yet. True: uploaded.
    std::vector<bool> target_status(scen.target_count, false);
    std::vector<double> data_remain(scen.target_count, 1.0);
    
    // Copy phones upload limits.
    std::vector<double> upload_limits(scen.phone_count, 0.0);
    for (int i = 0; i < scen.phone_count; ++i) {
      upload_limits[i] = scen.phones[i].upload_limit_;
    }
    
    // Simulate walk.
    nlog << "\n";
    nlog << "*********************************************\n";
    nlog << "Start simulated walk: \n";
    nlog << "*********************************************\n";
    
    for (int t = 0; t < scen.running_time; ++t) {
      nlog << "\n";
      nlog << "*********************************************\n";
      nlog << "Time: " << t << ":\n";
      const ThreeDimVector<int> &am = scen.adj_mats;
      const ThreeDimVector<double> &dm = scen.data_mats;
      for (int i = 0; i < scen.phone_count; ++i) {
        DataInfo_t &di = phone_datas;
        
        // Check if we can sense any target.
        for (int j = 0; j < scen.target_count; ++j) {
          // Target j's id in adj mat is phone count + j
          if (am(t, i, scen.phone_count + j) == 1 && target_status[j] == false) {
            if (di[i][j] < data_remain[j]) {
              // Target j is not fully uploaded and we do not
              // have full data in data storage.
              // Sense the target and save the data.
              nlog << "Phone " << i << " sense target " << j << ".\n";
              double sensing_cost = scen.phones[i].costs_.sensing_cost;
              r.AddCost(i, sensing_cost, Cost::SENSING);
              di[i][j] = data_remain[j];
            }
          }
        }
        
        bool empty_storage = true;
        for (int j = 0; j < scen.target_count; ++j) {
          if (di[i][j] > 0) {
            empty_storage = false;
            break;
          }
        }
        
        // Check if we have data that can be transferred.
        if (!empty_storage) {
          // There is data can be transferred.
          // Check if we can upload the data we have.
          for (int j = 0; j < scen.target_count; ++j) {
            assert(di[i][j] >= 0.0);
            if (di[i][j] == 0.0) continue;
            if (upload_limits[i] == 0.0) break;
            double upload_amount;
            if (di[i][j] > upload_limits[i]) {
              // Only be able to upload part of the data.
              upload_amount = upload_limits[i];
              upload_limits[i] = 0.0;
            } else {
              // Can upload all the data.
              // The corresponding target data is uploaded.
              upload_amount = di[i][j];
              upload_limits[i] -= di[i][j];
            }
            
            nlog << "Phone " << i << " uploads part of target " << j << ", upload amount: " << upload_amount << ".\n";
            assert(di[i][j] <= data_remain[j]);
            // Update global required data info.
            for (int k = 0; k < scen.phone_count; ++k) {
              if (di[k][j] > 0) {
                di[k][j] = (di[k][j] > upload_amount) ? (di[k][j] - upload_amount) : 0.0;
              }
            }
            data_remain[j] -= upload_amount;
            if (data_remain[j] == 0.0) {
              target_status[j] = true;
            }
            
            double upload_cost = scen.phones[i].costs_.upload_cost * upload_amount;
            r.AddCost(i, upload_cost, Cost::UPLOAD);
          }
          
          // We may have uploaded all the data.
          empty_storage = true;
          for (int j = 0; j < scen.target_count; ++j) {
            if (di[i][j] > 0) {
              empty_storage = false;
              break;
            }
          }
          
          if (empty_storage) {
            continue;
          }
          
          // Check if there is an available neighbor.
          double amount_transferred = 0.0;
          for (int j = 0; j < scen.phone_count; ++j) {
            if (i != j && am(t, i, j) == 1) {
              for (int k = 0; k < scen.target_count; ++k) {
                // Search on phone j to see if phone j already
                // has the data. Only transfer the data phone j
                // does not have.
                if (di[i][k] > di[j][k]) {
                  double data_transferred = di[i][k] - di[j][k];
                  double comm_cost1 = scen.phones[i].costs_.transfer_cost * data_transferred;
                  double comm_cost2 = scen.phones[j].costs_.transfer_cost * data_transferred;
                  di[j][k] = di[i][k];
                  amount_transferred += data_transferred;
                  r.AddCost(i, comm_cost1, Cost::COMM);
                  r.AddCost(j, comm_cost2, Cost::COMM);
                  nlog << "Phone " << i << " copy data of target " << k << " to phone " << j << ", transfer amount: " << data_transferred << ".\n";
                }
              }
              // break; // allow only one transfer.
            }
          } // End of for (int j
        } // End of if (!di.empty())
      } // End of for (int i
      
      // Check if all target datas are uploaded
      bool all_upload = true;
      for (int i = 0; i < scen.target_count; ++i) {
        if (target_status[i] == false) {
          all_upload = false;
          break;
        }
      }
      if (all_upload == true) {
        break;
        nlog << "All set. Exit loop.\n";
      }
      
    } // End of for (int t
    
    // Check if all target datas are uploaded
    bool all_upload = true;
    for (int i = 0; i < scen.target_count; ++i) {
      if (target_status[i] == false) {
        all_upload = false;
        break;
      }
    }
    if (!all_upload) {
      r.is_valid = false; // infeasible
      r.is_optimal = false;
    } else {
      r.is_valid = true; // valid
      r.is_optimal = true;
    }
    
    return r;
  }
}
