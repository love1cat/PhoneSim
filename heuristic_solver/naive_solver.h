//
//  naive_solver.h
//  MobileSensingSim
//
//  Created by Yuan on 5/19/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#ifndef __MobileSensingSim__naive_solver__
#define __MobileSensingSim__naive_solver__

#include "../solver_base.h"
#include <map>

namespace mobile_sensing_sim {
	// DataInfo
	// key (i, j), indicates how much percentage of data
	// of target j sensed by phone i has not been
	// uploaded.
	typedef std::map<std::pair<int, int>, double> DataInfo;
	
	class NaiveSolver : public SolverBase {
	public:
		virtual Result Solve(const Scenario& scen);
	};
}

#endif /* defined(__MobileSensingSim__naive_solver__) */
