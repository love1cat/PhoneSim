//
//  cplex_balance_adapter.cpp
//  PhoneSim
//
//  Created by Yuan on 11/11/14.
//  Copyright (c) 2014 Yuan. All rights reserved.
//

#include <boost/scoped_array.hpp>
#include "cplex_balance_adapter.h"

namespace mobile_sensing_sim {
  bool CplexBalanceAdapter::Solve(const Graph& g, const Scenario &scen, const BalanceOption &bo, Solution& s) {
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
    
    status_ = AddBalanceConstraints(g, scen, bo);
    if (status_) {
      fprintf (stderr, "Failed to add additional constraints as LP.\n");
      Reset();
      return false;
    }
    
    // Change problem type to MILP.
    if (use_milp_) {
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
    }
    
    //    status_ = CPXwriteprob (env_, lp_, "balance_prob.txt", "LP");
    
    if (use_milp_) {
      // Solve MILP.
      status_ = CPXmipopt (env_, lp_);
    } else {
      status_ = CPXchgprobtype (env_, lp_, CPXPROB_LP);
      
      status_ = CPXlpopt(env_, lp_);
    }
    
    if ( status_ ) {
      fprintf (stderr, "Failed to optimize LP.\n");
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
      fprintf (stderr,"No objective value available.  Exiting...\n");
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
    int narcs = cur_numcols - 1;
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
    
    
    s.is_valid = true;
    
    Reset();
    
    return true;
  }
  
  void CplexBalanceAdapter::Reset() {
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
  
  int CplexBalanceAdapter::AddBalanceConstraints(const Graph &g, const Scenario &scen, const BalanceOption &bo) {
    // Change the problem to MIN C
    // Add constraints for every phone that: total_cost - C <= 0
    
    // Add a new column corresponding to helper variable C
    int cur_numcols = CPXgetnumcols (env_, lp_);
    std::cout << "Number of cols before adding " << cur_numcols << std::endl;
    char ctype[1] = {'C'};
    double lb[1] = {0};
    double ub[1] = {CPX_INFBOUND};
    int ccnt = 1;
    double obj[1] = {0};
    
    status_ = CPXnewcols(env_, lp_, ccnt, obj, lb, ub, ctype, NULL);
    
    if (status_) {
      std::cout << "Error adding new cols... Status: " << status_ << std::endl;
      return status_;
    }
    
    
    // Add cost constraints for every phone.
    cur_numcols = CPXgetnumcols (env_, lp_);
    std::cout << "Number of cols after adding " << cur_numcols << std::endl;
    
    int cur_numrows = CPXgetnumrows (env_, lp_);
    std::cout << "Number of rows before adding " << cur_numrows << std::endl;
    
    assert(g.edge_count == cur_numcols - 1 && "The number of edges does not match the number of columns!");
    
    // Totally phone count new rows
    int rcnt = scen.phone_count;
    boost::scoped_array<double> rhs(new double[rcnt]);
    boost::scoped_array<char> sense(new char[rcnt]);
    for (int i = 0; i < rcnt; ++i) {
      rhs[i] = 0;
      sense[i] = 'L';
    }
    
    status_ = CPXnewrows(env_, lp_, rcnt, rhs.get(), sense.get(), NULL, NULL);
    
    if (status_) {
      std::cout << "Error adding new rows... Status: " << status_ << std::endl;
      return status_;
    }
    
    cur_numrows = CPXgetnumrows (env_, lp_);
    std::cout << "Number of rows after adding " << cur_numrows << std::endl;
    
    // Change coefficients based on graph
    std::vector<int> rowlist;
    std::vector<int> collist;
    std::vector<double> vallist;
    
    int count = 0;
    int start_id = cur_numrows - rcnt;
    for (int i = 0; i < g.edges.size(); ++i) {
      Edge e = g.edges[i];
      if(e.type == Edge::SRC_TO_TARGET ||
         e.type == Edge::PHONE_TO_SELF ||
         (e.type == Edge::TARGET_TO_PHONE && !bo.sensing) ||
         (e.type == Edge::PHONE_TO_PHONE && !bo.communication) ||
         (e.type == Edge::PHONE_TO_SINK && !bo.upload)) {
        continue;
      }
      
      int rowid = start_id + e.phone1_id;
      int colid = i;
      double val = e.cost;
      
      assert(rowid < cur_numrows && rowid >= 0 && "Invalid row id.");
      
      rowlist.push_back(rowid);
      collist.push_back(colid);
      vallist.push_back(val);
      ++count;
    }
    
    // Change coefficient of last variable
    for (int i = 0; i < rcnt; ++i) {
      int rowid = start_id + i;
      int colid = cur_numcols - 1;
      double val = -1;
      rowlist.push_back(rowid);
      collist.push_back(colid);
      vallist.push_back(val);
      ++count;
    }
    
    status_ = CPXchgcoeflist(env_, lp_, count, &rowlist[0], &collist[0], &vallist[0]);
    if (status_) {
      std::cout << "Error changing coef of new rows... Status: " << status_ << std::endl;
      return status_;
    }
    
    // Change objective.
    std::vector<double> values(cur_numcols, 0);
    values[cur_numcols-1] = 1;
    std::vector<int> indices;
    for (int i = 0; i < cur_numcols; ++i) {
      indices.push_back(i);
    }
    status_ = CPXchgobj(env_, lp_, cur_numcols, &indices[0], &values[0]);
    if (status_) {
      std::cout << "Error changing coef of obj... Status: " << status_ << std::endl;
      return status_;
    }
    
    return 0;
  }
}
