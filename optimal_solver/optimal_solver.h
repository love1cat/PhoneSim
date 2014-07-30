//
//  optimal_solver.h
//  MobileSensingSim
//
//  Created by Yuan on 5/5/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#ifndef __MobileSensingSim__optimal_solver__
#define __MobileSensingSim__optimal_solver__

#include "../solver_base.h"
#include "cplex_adapter.h"
#include "cplex_milp_adapter.h"

namespace mobile_sensing_sim {

	class OptimalSolver : public SolverBase {
	public:
		Solution Solve(const Scenario& scen);
		const GraphConverter& GetGraphConverter() {
			return gc_;
		}
	private:
//		CplexAdapter cplex_adapter_;
		CplexMILPAdapter cplex_adapter_;
		GraphConverter gc_;
	};
}

#endif /* defined(__MobileSensingSim__optimal_solver__) */
