//
//  solution.h
//  MobileSensingSim
//
//  Created by Yuan on 5/13/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#ifndef MobileSensingSim_solution_h
#define MobileSensingSim_solution_h

#include <vector>

namespace mobile_sensing_sim {
	struct Solution {
		Solution() : is_valid(false) {}
		void Clear() {
			edge_values.clear();
			edge_costs.clear();
		}
		bool is_valid;
		double obj;
		int solution_status;
		int edge_count;
		int vertex_count;
		std::vector<double> edge_values;
		std::vector<double> edge_costs;
	};
}

#endif
