//
//  cplex_milp_adapter.cpp
//  MobileSensingSim
//
//  Created by Yuan on 5/21/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#include <boost/scoped_array.hpp>
#include "cplex_milp_adapter.h"

namespace mobile_sensing_sim {
	bool CplexMILPAdapter::Solve(const Graph& g, Solution& s) {
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
		
		//*******************************************
		//Convert to Mixed Interger LP problem
		//*******************************************
		
		/* Create LP object*/
		
		lp_ = CPXcreateprob (env_, &status_, "netex2");
		if ( lp_ == NULL ) {
			fprintf (stderr, "Failed to create LP object.\n");
			Reset();
			return false;
		}
		
		/* Copy LP representation of network problem to lp object, along
		 with the current basis available in the network object. */
		
		status_ = CPXcopynettolp (env_, lp_, net_);
		if ( status_ ) {
			fprintf (stderr, "Failed to copy network as LP.\n");
			Reset();
			return false;
		}
		
		// Change problem type to MILP.
		status_ = CPXchgprobtype (env_, lp_, CPXPROB_MILP);
		
		// Create variable type array.
		boost::scoped_array<char> ctype(new char[g.edge_count]);
		for (int i = 0; i < g.edge_count; ++i) {
			if (g.edges[i].type == Edge::TARGET_TO_PHONE) {
				ctype[i] = 'B';
			} else {
				ctype[i] = 'C';
			}
		}
		
		// Set variable type in MILP.
		status_ = CPXcopyctype (env_, lp_, ctype.get());
		
		// Solve MILP.
		status_ = CPXmipopt (env_, lp_);
		if ( status_ ) {
			fprintf (stderr, "Failed to optimize MIP.\n");
			Reset();
			return false;
		}
		
		// Optimal status is 101
		int solstat = CPXgetstat (env_, lp_);
		
		/* Write the output to the screen. */
		
		printf ("\nSolution status = %d\n", solstat);
		
		double objval;
		status_ = CPXgetobjval (env_, lp_, &objval);
		if ( status_ ) {
			fprintf (stderr,"No MIP objective value available.  Exiting...\n");
			Reset();
			return false;
		}
		s.obj = objval;
		
		printf ("Solution value  = %f\n\n", objval);
		
		/* The size of the problem should be obtained by asking CPLEX what
		 the actual size is, rather than using what was passed to CPXcopylp.
		 cur_numrows and cur_numcols store the current number of rows and
		 columns, respectively.  */
		
		int cur_numrows = CPXgetnumrows (env_, lp_);
		int cur_numcols = CPXgetnumcols (env_, lp_);
		
		boost::scoped_array<double> x(new double[g.edge_count]);
		
		status_ = CPXgetx (env_, lp_, x.get(), 0, cur_numcols-1);
		if ( status_ ) {
			fprintf (stderr, "Failed to get optimal integer x.\n");
			Reset();
			return false;
		}
		
		/* Write the output to the Solution vector */
		s.obj = objval;
		s.solution_status = solstat;
		int narcs = cur_numcols;
		int nnode = cur_numrows;
		
		s.edge_values.resize(narcs);
		s.edge_count = narcs;
		s.vertex_count = nnode;
		
		std::copy(x.get(), x.get() + narcs, s.edge_values.begin());
		
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
		
		s.is_valid = true;
		
		Reset();
		
		return true;
	}
	
	int CplexMILPAdapter::BuildNetwork(const Graph& g) {
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
	
	bool CplexMILPAdapter::Reset() {
		/* Free up the problem as allocated by CPXNETcreateprob, if necessary */
		
		if ( net_ != NULL ) {
			status_ = CPXNETfreeprob (env_, &net_);
			if ( status_ ) {
				fprintf (stderr, "CPXNETfreeprob failed, error code %d.\n", status_);
			}
		}
		
		/* Free up the problem as allocated by CPXcreateprob, if necessary */
		
		if ( lp_ != NULL ) {
			status_ = CPXfreeprob (env_, &lp_);
			if ( status_ ) {
				fprintf (stderr, "CPXfreeprob failed, error code %d.\n", status_);
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
