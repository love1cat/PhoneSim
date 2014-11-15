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
    Reset();
    
    CreateNetworkProblem(g);
		
		//*******************************************
		//Convert to Mixed Interger LP problem
		//*******************************************
    
    s.Clear();
    s.is_valid = false;
    
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
	
	void CplexMILPAdapter::Reset() {
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
	}
}
