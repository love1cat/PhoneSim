//
//  agg_heuristic_solver.h
//  PhoneSim
//
//  Created by Yuan on 7/11/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#ifndef __PhoneSim__agg_heuristic_solver__
#define __PhoneSim__agg_heuristic_solver__

#include "../solver_base.h"
#include "../optimal_solver/cplex_milp_adapter.h"
#include "../optimal_solver/cplex_adapter.h"

namespace mobile_sensing_sim {
	class AggressiveHeuristicSolver : public SolverBase {
	public:
		AggressiveHeuristicSolver(const int report_period) : report_period_(report_period) {
		}
		virtual Result Solve(const Scenario& scen);
	private:
		CplexMILPAdapter cplex_milp_adapter_;
		CplexAdapter cplex_adapter_;
		const int report_period_;
	};
}

#endif /* defined(__PhoneSim__agg_heuristic_solver__) */
