//
//  heuristic_solver.h
//  MobileSensingSim
//
//  Created by Yuan on 5/12/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#ifndef __MobileSensingSim__heuristic_solver__
#define __MobileSensingSim__heuristic_solver__

#include "../scenario_generator/scenario_generator.h"
#include "../optimal_solver/cplex_milp_adapter.h"
#include "../optimal_solver/cplex_adapter.h"

namespace mobile_sensing_sim {
	class HeuristicSolver {
	public:
		HeuristicSolver(const int report_period) : report_period_(report_period) {}
		Solution Solve(const Scenario& scen);
	private:
		CplexMILPAdapter cplex_milp_adapter_;
		CplexAdapter cplex_adapter_;
		const int report_period_;
	};
}

#endif /* defined(__MobileSensingSim__heuristic_solver__) */
