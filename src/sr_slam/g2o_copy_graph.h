/*
 *  Jan. 29, 2016, David Z
 *
 *  provide a function that copy a graph to another one, with type conversion 
 *
 * */


#ifndef G2O_COPY_GRAPH_H
#define G2O_COPY_GRAPH_H

#include "g2o/core/sparse_optimizer.h"

using namespace std;
namespace g2o{

  template<typename VFrom, typename EFrom, typename VTo, typename ETo>
  extern bool copyGraph(SparseOptimizer* gFrom, SparseOptimizer* gTo); 
 

  template<typename VF, typename VT>
  extern VT* copyNode(VF* );

  template<typename EF, typename ET>
  extern ET* copyEdge(EF*, SparseOptimizer* g);
}

#include "g2o_copy_graph.hpp"


#endif
