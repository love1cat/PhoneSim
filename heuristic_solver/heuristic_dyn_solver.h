//
//  heuristic_dyn_solver.h
//  MobileSensingSim
//
//  Created by Yuan on 5/12/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#ifndef __MobileSensingSim__heuristic_dyn_solver__
#define __MobileSensingSim__heuristic_dyn_solver__

#include "../solver_base.h"
#include "../optimal_solver/cplex_milp_adapter.h"
#include "../optimal_solver/cplex_adapter.h"
#include "../optimal_solver/cplex_balance_adapter.h"

namespace mobile_sensing_sim {
	class HeuristicDynSolver : public SolverBase{
	public:
		HeuristicDynSolver(const int report_period, double multiple = 2) : report_period_(report_period), multiple_(multiple){}
		Result Solve(const Scenario& scen);
	private:
    virtual void IncreaseCost(Phone &p) const;
		CplexMILPAdapter cplex_milp_adapter_;
    CplexBalanceAdapter cplex_balance_adapter_;
    
		const int report_period_;
    double multiple_;
	};
}

#endif /* defined(__MobileSensingSim__heuristic_dyn_solver__) */
