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
#include "cplex_adapter_base.h"
#include "graph_converter.h"
#include "../solution.h"

namespace mobile_sensing_sim {
  class CplexMILPAdapter : public CplexAdapterBase {
	public:
		CplexMILPAdapter() : lp_(NULL){}
		bool Solve(const Graph &g, Solution &s);
	private:
		void Reset();

		CPXLPptr  lp_;
	};
}

#endif /* defined(__MobileSensingSim__cplex_milp_adapter__) */
