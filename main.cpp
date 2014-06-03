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
#include "optimal_solver/optimal_solver.h"
#include "heuristic_solver/heuristic_solver.h"
#include "heuristic_solver/naive_solver.h"

namespace mss = mobile_sensing_sim;

mss::MonitorMap CreateMap() {
	// Length and width.
	int length = 500;
	int width = 500;
	
	// Entry points.
	std::vector<mss::Point> entry_points;
	entry_points.push_back(mss::Point(0, 250));
	entry_points.push_back(mss::Point(250, 0));
	entry_points.push_back(mss::Point(250, 500));
	entry_points.push_back(mss::Point(500, 250));
	
	// Intersect points.
	std::vector<mss::Point> intersect_points;
	intersect_points.push_back(mss::Point(250, 250));
	
	// Create area map.
	mss::AreaMap am(entry_points, intersect_points, length, width);
	
	// Monitor / target points.
	std::vector<mss::Point> monitor_points;
	monitor_points.push_back(mss::Point(125, 250));
	monitor_points.push_back(mss::Point(275, 250));
	monitor_points.push_back(mss::Point(250, 375));
	monitor_points.push_back(mss::Point(250, 125));
	
	monitor_points.push_back(mss::Point(180, 250));
	monitor_points.push_back(mss::Point(220, 250));
	monitor_points.push_back(mss::Point(250, 320));
	monitor_points.push_back(mss::Point(250, 180));
	// Create monitor map.
	return mss::MonitorMap(monitor_points, am);
}



int main(int argc, const char * argv[])
{
	// Scenario parameters.
	mss::ScenarioParameters sp;
	sp.sensing_range = 40;
	sp.comm_range = 40;
	sp.running_time = 1000;
	//	sp.phone_count = 40;
	sp.speed_range = mss::Range(5, 15, 0.1);
	sp.start_time_range = mss::Range(0, 100);
	sp.seed = 0;
	sp.map = CreateMap();
	sp.data_per_second = 0.5;
	sp.sensing_cost_range = mss::Range(2, 6, 0.5);
	sp.transfer_cost_range = mss::Range(2, 6, 0.5);
	sp.upload_cost_range = mss::Range(2, 6, 0.5);
	sp.upload_limit_range = mss::Range(1, 5, 0.1);
	
	int phone_counts[] = {50};
	const int kPhoneCountsSize = 1;
	std::ofstream outfile("diff_phone_counts.txt");
    std::ofstream statusfile("algorithm_status.txt");
	for (int i = 0; i < kPhoneCountsSize; ++i) {
		sp.phone_count = phone_counts[i];
		outfile << phone_counts[i] << "\t";
        statusfile << phone_counts[i] << "\t";
		
		// Create scneario generator.
		mss::ScenarioGenerator sg(sp);
		
		// Generate scenario.
		const mss::Scenario& scen = sg.GenerateDefaultScenario();
		
		// Optimal solver
		mss::OptimalSolver os;
		mss::Solution opt_s = os.Solve(scen);
		std::cout << "Optimal obj value: " << opt_s.obj << std::endl;
		outfile << opt_s.obj << "\t";
        statusfile << opt_s.solution_status << "\t";

		
    	// Heuristic solver
        int report_periods[] = {120};
        const int kReportPeriodCounts = 1;
        for (int j = 0; j < kReportPeriodCounts; ++j) {
        	mss::HeuristicSolver hs(report_periods[j]);
        	mss::Solution h_s = hs.Solve(scen);
        	std::cout << "Heuristic obj value: " << h_s.obj << std::endl;
        	outfile << h_s.obj << "\t";
            statusfile << h_s.solution_status << "\t";
        }
		
		// Naive solver
		mss::NaiveSolver ns;
		mss::Solution n_s = ns.Solve(scen);
		std::cout << "Naive obj value: " << n_s.obj << std::endl;
		outfile << n_s.obj << "\t";
        statusfile << n_s.solution_status << "\t";
		
		outfile << "\n";
        statusfile << "\n";
	}
	outfile.close();
    statusfile.close();
	
    return 0;
}

