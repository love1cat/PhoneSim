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
#include "cplex_adapter_base.h"
#include "graph_converter.h"

namespace mobile_sensing_sim {
  class CplexAdapter : public CplexAdapterBase {
	public:
		bool Solve(const Graph &g, Solution &s);
	private:
		static std::string network_file_name_;
	};
}

#endif /* defined(__MobileSensingSim__cplex_adapter__) */
