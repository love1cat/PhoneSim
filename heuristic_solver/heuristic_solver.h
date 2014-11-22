//
//  heuristic_solver.h
//  MobileSensingSim
//
//  Created by Yuan on 5/12/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#ifndef __MobileSensingSim__heuristic_solver__
#define __MobileSensingSim__heuristic_solver__

#include "../solver_base.h"
#include "../optimal_solver/cplex_milp_adapter.h"
#include "../optimal_solver/cplex_adapter.h"
#include "../optimal_solver/cplex_balance_adapter.h"

namespace mobile_sensing_sim {
	class HeuristicSolver : public SolverBase{
	public:
		HeuristicSolver(const int report_period, bool use_balance = false) : report_period_(report_period), use_balance_(use_balance) {}
		Result Solve(const Scenario& scen);
	private:
		CplexMILPAdapter cplex_milp_adapter_;
		CplexAdapter cplex_adapter_;
    CplexBalanceAdapter cplex_balance_adapter_;
    
		const int report_period_;
    bool use_balance_;
	};
}

#endif /* defined(__MobileSensingSim__heuristic_solver__) */
