
namespace g2o{

template<typename VFrom, typename EFrom, typename VTo, typename ETo>
bool copyGraph(SparseOptimizer* gf, SparseOptimizer* gt)
{
  // 1, copy node first 
  for(HyperGraph::VertexIDMap::iterator it = gf->vertices().begin(); it!= gf->vertices().end(); ++it)
  {
    VFrom* vin = dynamic_cast<VFrom*>(it->second); 
    if(vin == NULL)
      continue;
    VTo* vout = copyNode<VFrom, VTo>(vin); 
    gt->addVertex(vout); 
  }
  // 2, copy edge second 
  for(HyperGraph::EdgeSet::iterator it = gf->edges().begin(); 
      it != gf->edges().end(); ++it)
  {
    EFrom* ein = dynamic_cast<EFrom*>(*it); 
    if(ein == NULL) 
      continue;
    ETo* eout = copyEdge<EFrom, ETo>(ein, gt); 
    gt->addEdge(eout);
  }
  return true;
}

template<typename VF, typename VT>
VT* copyNode(VF* pin)
{
  VT* pr = new VT; 
  pr->setId(pin->id()); 
  pr->setEstimate(pin->estimate()); 
  pr->setFixed(pin->fixed());
  return pr;
}

template<typename EF, typename ET>
ET* copyEdge(EF* pin, SparseOptimizer* g)
{
  ET * er = new ET; 
  int id1 = (pin->vertices()[0])->id(); 
  int id2 = (pin->vertices()[1])->id(); 
  er->vertices()[0] = g->vertex(id1); 
  er->vertices()[1] = g->vertex(id2); 
  er->setMeasurement(pin->measurement());
  er->setInformation(pin->information()); 
  return er;
}

}
