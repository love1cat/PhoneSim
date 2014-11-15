//
//  cplex_adapter_base.cpp
//  PhoneSim
//
//  Created by Yuan on 11/4/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#include "cplex_adapter_base.h"

namespace mobile_sensing_sim {
  bool CplexAdapterBase::CreateNetworkProblem(const Graph &g) {
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
    
    return true;
  }
  
  int CplexAdapterBase::BuildNetwork(const Graph& g) {
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
  
  void CplexAdapterBase::Reset() {
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
  }
}