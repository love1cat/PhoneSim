//
//  cplex_adapter.h
//  MobileSensingSim
//
//  Created by Yuan on 5/10/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#ifndef __MobileSensingSim__cplex_adapter__
#define __MobileSensingSim__cplex_adapter__

#include <vector>
#include <ilcplex/cplex.h>
#include "graph_converter.h"
#include "../solution.h"

namespace mobile_sensing_sim {
	class CplexAdapter {
	public:
		CplexAdapter() : env_(NULL), net_(NULL){}
		bool Solve(const Graph &g, Solution &s, bool write_file = false);
	private:
		int BuildNetwork(const Graph& g);
		bool Reset();
		
		int status_;
		
		static std::string network_file_name_;
		
		CPXENVptr env_;
		CPXNETptr net_;
	};
}

#endif /* defined(__MobileSensingSim__cplex_adapter__) */
