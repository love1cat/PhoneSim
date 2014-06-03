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
	
	bool CplexAdapter::Solve(const Graph& g, Solution& s, bool write_file) {
		/* Declare variables and arrays for retrieving problem data and
		 solution information later on. */
		
		Reset();
		s.Clear();
		s.is_valid = false;
		
		/* Initialize the CPLEX environment */
		
		env_ = CPXopenCPLEX (&status_);
		
		/* If an error occurs, the status_ value indicates the reason for
		 failure.  A call to CPXgeterrorstring will produce the text of
		 the error message.  Note that CPXopenCPLEX produces no
		 output, so the only way to see the cause of the error is to use
		 CPXgeterrorstring.  For other CPLEX routines, the errors will
		 be seen if the CPX_PARAM_SCRIND indicator is set to CPX_ON.  */
		
		if ( env_ == NULL ) {
			char  errmsg[CPXMESSAGEBUFSIZE];
			fprintf (stderr, "Could not open CPLEX environment.\n");
			CPXgeterrorstring (env_, status_, errmsg);
			fprintf (stderr, "%s", errmsg);
			Reset();
			return false;
		}
		
		/* Turn on output to the screen */
		
		status_ = CPXsetintparam (env_, CPX_PARAM_SCRIND, CPX_ON);
		if ( status_ ) {
			fprintf (stderr,
					 "Failure to turn on screen indicator, error %d.\n", status_);
			Reset();
			return false;
		}
		
		/* Create the problem. */
		
		net_ = CPXNETcreateprob (env_, &status_, "netex1");
		
		/* A returned pointer of NULL may mean that not enough memory
		 was available or there was some other problem.  In the case of
		 failure, an error message will have been written to the error
		 channel from inside CPLEX.  In this example, the setting of
		 the parameter CPX_PARAM_SCRIND causes the error message to
		 appear on stdout.  */
		
		if ( net_ == NULL ) {
			fprintf (stderr, "Failed to create network object.\n");
			Reset();
			return false;
		}
		
		/* Fill in the data for the problem.  Note that since the space for
		 the data already exists in local variables, we pass the arrays
		 directly to the routine to fill in the data structures.  */
		
		status_ = BuildNetwork(g);
		
		if ( status_ ) {
			fprintf (stderr, "Failed to build network problem.\n");
			Reset();
			return false;
		}
		
		
		/* Optimize the problem and obtain solution. */
		
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
		
		
		//		x     = (double *) malloc (narcs  * sizeof (double));
		//		dj    = (double *) malloc (narcs  * sizeof (double));
		//		pi    = (double *) malloc (nnodes * sizeof (double));
		//		slack = (double *) malloc (nnodes * sizeof (double));
		
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
		
		if (write_file) {
			status_ = CPXNETwriteprob (env_, net_, network_file_name_.c_str(), NULL);
			if ( status_ ) {
				fprintf (stderr, "Failed to write network to disk.\n");
				Reset();
				return false;
			}
		}
		
		s.is_valid = true;
		
		Reset();
		
		return true;
	}
	
	int CplexAdapter::BuildNetwork(const Graph& g) {
		status_ = 0;
		
		/* Delete existing network.  This is not necessary in this
		 context since we know we have an empty network object.
		 Notice that CPXNETdelnodes deletes all arcs incident to
		 the deleted nodes as well.  Therefore this one function
		 call effectively deletes an existing network problem. */
		
		if ( CPXNETgetnumnodes (env_, net_) > 0 ) {
			status_ = CPXNETdelnodes (env_, net_, 0,
									  CPXNETgetnumnodes (env_, net_)-1);
			if ( status_ ) {
				return status_;
			};
		}
		
		/* Set optimization sense */
		
		status_ = CPXNETchgobjsen (env_, net_, CPX_MIN);
		if ( status_ ) {
			return status_;
		};
		
		/* Add nodes to network along with their supply values,
		 but without any names. */
		
		status_ = CPXNETaddnodes (env_, net_, g.vertex_count, &g.vertex_supply[0], NULL);
		if ( status_ ) {
			return status_;
		};
		
		/* Add arcs to network along with their objective values and
		 bounds, but without any names. */
		
		status_ = CPXNETaddarcs (env_, net_, g.edge_count, &g.edge_tails[0], &g.edge_heads[0], &g.edge_capacity_lower_bounds[0], &g.edge_capacity_uppper_bounds[0], &g.edge_costs[0], NULL);
		if ( status_ ) {
			return status_;
		};
		
		return 0;
	}
	
	bool CplexAdapter::Reset() {
		/* Free up the problem as allocated by CPXNETcreateprob, if necessary */
		
		if ( net_ != NULL ) {
			status_ = CPXNETfreeprob (env_, &net_);
			if ( status_ ) {
				fprintf (stderr, "CPXNETfreeprob failed, error code %d.\n", status_);
			}
		}
		
		/* Free up the CPLEX environment, if necessary */
		
		if ( env_ != NULL ) {
			status_ = CPXcloseCPLEX (&env_);
			
			/* Note that CPXcloseCPLEX produces no output,
			 so the only way to see the cause of the error is to use
			 CPXgeterrorstring.  For other CPLEX routines, the errors will
			 be seen if the CPX_PARAM_SCRIND indicator is set to CPX_ON. */
			
			if ( status_ ) {
				char  errmsg[CPXMESSAGEBUFSIZE];
				fprintf (stderr, "Could not close CPLEX environment.\n");
				CPXgeterrorstring (env_, status_, errmsg);
				fprintf (stderr, "%s", errmsg);
			}
		}
		
		return true;
	}
}