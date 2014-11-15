//
//  cplex_adapter.cpp
//  MobileSensingSim
//
//  Created by Yuan on 5/10/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#include <boost/scoped_array.hpp>
#include "cplex_adapter.h"

namespace mobile_sensing_sim {
	std::string CplexAdapter::network_file_name_ = "cplex_network.net";
	
	bool CplexAdapter::Solve(const Graph& g, Solution& s) {
    Reset();
    
    CreateNetworkProblem(g);
		
		/* Optimize the problem and obtain solution. */
    s.Clear();
    s.is_valid = false;
		
		status_ = CPXNETprimopt (env_, net_);
		if ( status_ ) {
			fprintf (stderr, "Failed to optimize network.\n");
			Reset();
			return false;
		}
		
		/* get network dimensions */
		
		const int narcs  = CPXNETgetnumarcs  (env_, net_);
		const int nnodes = CPXNETgetnumnodes (env_, net_);
		
		assert(g.edge_count == narcs);
		
		/* allocate memory for solution data */
		boost::scoped_array<double> x(new double[narcs]);
		boost::scoped_array<double> dj(new double[narcs]);
		boost::scoped_array<double> pi(new double[nnodes]);
		boost::scoped_array<double> slack(new double[nnodes]);
		
		if ( x     == NULL ||
			dj    == NULL ||
			pi    == NULL ||
			slack == NULL   ) {
			fprintf (stderr, "Failed to allocate arrays.\n");
			Reset();
			return false;
		}
		
		double objval;
		int solstat;
		status_ = CPXNETsolution (env_, net_, &solstat, &objval, x.get(), pi.get(), slack.get(), dj.get());
		if ( status_ ) {
			fprintf (stderr, "Failed to obtain solution.\n");
			Reset();
			return false;
		}
		
		/* Write the output to the Solution vector */
		s.edge_count = narcs;
		s.vertex_count = nnodes;
		s.obj = objval;
		s.solution_status = solstat;
		
		s.edge_values.resize(narcs);
		s.edge_costs.resize(narcs);
		
		std::copy(x.get(), x.get() + narcs, s.edge_values.begin());
		std::copy(dj.get(), dj.get() + narcs, s.edge_costs.begin());
		
		//		/* Write the output to the screen. */
		//
		//		printf ("\nSolution status = %d\n", solstat);
		//		printf ("Solution value  = %f\n\n", objval);
		//
		//		for (i = 0; i < nnodes; i++) {
		//			printf ("Node %2d:  Slack = %10f  Pi = %10f\n", i, slack[i], pi[i]);
		//		}
		//
		//		for (j = 0; j < narcs; j++) {
		//			printf ("Arc  %2d:  Value = %10f  Reduced cost = %10f\n",
		//					j, x[j], dj[j]);
		//		}
		
		/* Finally, write a copy of the problem to a file. */
		
//		if (write_file) {
//			status_ = CPXNETwriteprob (env_, net_, network_file_name_.c_str(), NULL);
//			if ( status_ ) {
//				fprintf (stderr, "Failed to write network to disk.\n");
//				Reset();
//				return false;
//			}
//		}
		
		s.is_valid = true;
		
		Reset();
		
		return true;
	}
}