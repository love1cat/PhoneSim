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
	Solution NaiveSolver::Solve(const Scenario &scen) {
        nlog.Reset();
		Solution s;
		
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
		s.obj = 0.0;
        s.sensing_cost = 0.0;
        s.comm_cost = 0.0;
        s.upload_cost = 0.0;
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
                            s.sensing_cost += sensing_cost;
							s.obj += sensing_cost;
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
                            s.upload_cost += upload_cost;
							s.obj += upload_cost;
							
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
                            s.upload_cost += upload_cost;
							s.obj += upload_cost;
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
                                    double comm_cost = (scen.phones[i].costs_.transfer_cost + scen.phones[j].costs_.transfer_cost) * data_required[(*it).first];
                                    s.comm_cost += comm_cost;
									s.obj += comm_cost;
									phone_datas[j][(*it).first] = data_required[(*it).first];
									amount_transferred += data_required[(*it).first];
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
			s.solution_status = 3; // infeasible
		} else {
			s.solution_status = 1; // valid
		}
		
		return s;
	}
}
