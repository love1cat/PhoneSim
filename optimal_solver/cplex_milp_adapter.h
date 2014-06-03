//
//  cplex_milp_adapter.h
//  MobileSensingSim
//
//  Created by Yuan on 5/21/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#ifndef __MobileSensingSim__cplex_milp_adapter__
#define __MobileSensingSim__cplex_milp_adapter__

#include <vector>
#include <ilcplex/cplex.h>
#include "graph_converter.h"
#include "../solution.h"

namespace mobile_sensing_sim {
	class CplexMILPAdapter {
	public:
		CplexMILPAdapter() : env_(NULL), net_(NULL), lp_(NULL){}
		bool Solve(const Graph &g, Solution &s);
	private:
		int BuildNetwork(const Graph& g);
		bool Reset();
		
		int status_;
		
		CPXENVptr env_;
		CPXNETptr net_;
		CPXLPptr  lp_;
	};
}

#endif /* defined(__MobileSensingSim__cplex_milp_adapter__) */
