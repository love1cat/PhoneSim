//
//  solver_base.h
//  PhoneSim
//
//  Created by Yuan on 7/9/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#ifndef __MobileSensingSim__solver_base__
#define __MobileSensingSim__solver_base__

#include "scenario_generator/scenario_generator.h"
#include "solution.h"

namespace mobile_sensing_sim {
	class SolverBase {
	public:
		virtual Solution Solve(const Scenario& scen) = 0;
	};
}

#endif /* defined(__MobileSensingSim__solver_base__) */
