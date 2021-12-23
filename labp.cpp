#include"labp.hpp"
#include<iostream>
#include<cmath>
#include<algorithm>
#include <set>
#include<memory>
#include <queue>
#include <functional>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/max_cardinality_matching.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <ortools/graph/assignment.h>

using Graph =boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS>;
using Vertex=boost::graph_traits<Graph>::vertex_descriptor;
using CostMatrix=std::vector<std::vector<double>>;

bool check_feasible(CostMatrix &costMatrix,std::vector<int>&assignment,double threshold){
 
    int m=costMatrix.size();
    Graph g(2*m);
    //(i,j)==>i*m+j
    for(int i=0;i<m;i++){
        for(int j=0;j<m;j++){
            if(costMatrix[i][j]<=threshold)add_edge(i,j+m,g);
        }
    }
    std::vector< Vertex> mate(2*m);
    bool success=true;
    boost::edmonds_maximum_cardinality_matching(g, &mate[0]);
    if(boost::matching_size(g,&mate[0])!=m) success=false;
    if(success==true){
        
        boost::graph_traits< Graph >::vertex_iterator vi, vi_end;
        for (boost::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi)
            if (mate[*vi] != boost::graph_traits< Graph >::null_vertex()
                && *vi < mate[*vi])
                assignment[*vi]=mate[*vi]-m;
    }

    return success;
}







double labp_solve(CostMatrix& costMatrix, std::vector<int>& assignment){
  
    std::vector<double> open;
    int m=costMatrix.size();
    const int max_inf=1e6;
    if(assignment.size()!=m)assignment.resize(m);
    std::vector<int> last_good_assignment;

    for(int i=0;i<costMatrix.size();i++){
        for(int j=0;j<costMatrix[i].size();j++){
    
            open.push_back(costMatrix[i][j]);
            // std::make_heap(open.begin(),open.end(),std::greater<>{});
        }
        // std::cout<<std::endl;
    }
    std::sort(open.begin(),open.end());

    
    int lb=0;
    int hb=open.size()-1;
    int thresh_id=hb;
    bool infeasible=true;
    while(hb>=lb){
        thresh_id=lb+(hb-lb)/2;
        double threshold=open[thresh_id];
        //std::cout<<threshold<<" thresh_id="<<thresh_id<<"<lb,hb>="<<lb<<","<<hb<<std::endl;
        bool success=check_feasible(costMatrix,assignment,threshold);
        if(success==true) {
            hb=thresh_id-1;
            last_good_assignment=assignment;
            infeasible=false;
        }
        else lb=thresh_id+1;
    }
    // std::cout<<thresh_id<<" "<<lb<<" "<<hb<<std::endl;
    if(infeasible){
        // std::cout<<"matrix infeasible!"<<std::endl;
        throw std::runtime_error("matrix infeasible!");
        // return -1;
    }
    // std::cout<<"Solved "<<assignment.size()<<std::endl;
    assignment=last_good_assignment;
    //std::cout<<"debug "<<lb<<" "<<hb<<std::endl;
    return open[hb];


}


bool check_feasible_sparse(std::vector<std::tuple<int,int,double>>&costEdges,std::vector<int>&assignment,std::set<int> &indexes,double threshold){
    int m=indexes.size();
    // std::cout<<"m size="<<m<<std::endl;
    Graph g(2*m);
    //(i,j)==>i*m+j
    // for(int i=0;i<m;i++){
    //     for(int j=0;j<m;j++){
    //         if(costMatrix[i][j]<=threshold)add_edge(i,j+m,g);
    //     }
    // }
    for(auto &edge:costEdges){
        if(std::get<2>(edge)<=threshold) add_edge(std::get<0>(edge),std::get<1>(edge)+m,g);
    }

    std::vector< Vertex> mate(2*m);
    bool success=true;
    boost::edmonds_maximum_cardinality_matching(g, &mate[0]);
    if(boost::matching_size(g,&mate[0])!=m) success=false;
    if(success==true){
        
        boost::graph_traits< Graph >::vertex_iterator vi, vi_end;
        for (boost::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi)
            if (mate[*vi] != boost::graph_traits< Graph >::null_vertex()
                && *vi < mate[*vi])
                assignment[*vi]=mate[*vi]-m;
    }

    return success;
}

double lba_sparse(std::vector<std::tuple<int,int,double>>&costEdges, std::vector<int>& assignment){
    using weighted_edge=std::tuple<int,int,double>;
    
    auto compare=[](weighted_edge e1,weighted_edge e2){
        return std::get<2>(e1) <  std::get<2>(e2);
    };
    std::sort(costEdges.begin(),costEdges.end(),compare);
    std::set<int> indexes;
    for(auto &edge:costEdges){
        indexes.insert(std::get<0>(edge));
        // std::cout<<std::get<0>(edge)<<" "<<std::get<1>(edge)<<" "<<std::get<2>(edge)<<" \n";
    }
    int m=indexes.size();
    if(assignment.size()!=m)assignment.resize(m);
    int lb=0;
    int hb=costEdges.size()-1;
    int thresh_id=hb;
    bool infeasible=true;
    std::vector<int> last_good_assignment;
    while(hb>=lb){
        thresh_id=lb+(hb-lb)/2;
        double threshold=std::get<2>(costEdges[thresh_id]);
        // std::cout<<threshold<<" before thresh_id="<<thresh_id<<" <lb,hb>="<<lb<<","<<hb<<std::endl;
        bool success=check_feasible_sparse(costEdges,assignment,indexes,threshold);
        if(success==true) {
            hb=thresh_id-1;
            last_good_assignment=assignment;
            infeasible=false;
        }
        else lb=thresh_id+1;
        // std::cout<<threshold<<"after  thresh_id="<<thresh_id<<" <lb,hb>="<<lb<<","<<hb<<std::endl;
    }
    // std::cout<<thresh_id<<" "<<lb<<" "<<hb<<std::endl;
    if(infeasible){
        // std::cout<<"matrix infeasible!"<<std::endl;
        throw std::runtime_error("matrix infeasible!");
        // return -1;
    }
    // std::cout<<"Solved "<<assignment.size()<<std::endl;
    assignment=last_good_assignment;
    //std::cout<<"debug "<<lb<<" "<<hb<<std::endl;
    return 1;
}


using costMatrix=std::vector<std::vector<double>>;
/*This function is the jv shortest augmenting path algorithm to solve the assignment problem*/
double lap(int dim,
        costMatrix &assigndouble,
        std::vector<int>& rsol,
        std::vector<int> &csol,
        std::vector<double> &u,
        std::vector<double> &v)

// input:
// dim        - problem size
// assigndouble - double matrix

// output:
// rsol     - intumn assigned to int in solution
// csol     - int assigned to intumn in solution
// u          - dual variables, int reduction numbers
// v          - dual variables, intumn reduction numbers

{
  bool unassignedfound;
  int  i, imin, numfree = 0, prvnumfree, f, i0, k, freeint;
  int  j, j1, j2, endofpath, last, low, up;
  double min, h, umin, usubmin, v2;
  std::vector<int> free(dim);
  std::vector<int> intlist(dim);
  std::vector<int> matches(dim);
  std::vector<double> d(dim);
  std::vector<int> pred(dim);


  // init how many times a int will be assigned in the intumn reduction.
  for (i = 0; i < dim; i++)
    matches[i] = 0;

  // intUMN REDUCTION
  for (j = dim;j--;) // reverse order gives better results.
  {
    // find minimum double over ints.
    min = assigndouble[0][j];
    imin = 0;
    for (i = 1; i < dim; i++)
      if (assigndouble[i][j] < min)
      {
        min = assigndouble[i][j];
        imin = i;
      }
    v[j] = min;
    if (++matches[imin] == 1)
    {
      // init assignment if minimum int assigned for first time.
      rsol[imin] = j;
      csol[j] = imin;
    }
    else if(v[j]<v[rsol[imin]]){
        int j1 = rsol[imin];
        rsol[imin] = j;
        csol[j] = imin;
        csol[j1] = -1;
    }
    else
      csol[j] = -1;        // int already assigned, intumn not assigned.
  }

  // REDUCTION TRANSFER
  for (i = 0; i < dim; i++)
    if (matches[i] == 0)     // fill list of unassigned 'free' ints.
      free[numfree++] = i;
   else
      if (matches[i] == 1)   // transfer reduction from ints that are assigned once.
      {
        j1 = rsol[i];
        min = BIG;
        for (j = 0; j < dim; j++)
          if (j != j1)
            if (assigndouble[i][j] - v[j] < min)
              min = assigndouble[i][j] - v[j];
        v[j1] = v[j1] - min;
      }

    //   AUGMENTING int REDUCTION
  int loopcnt = 0;           // do-loop to be done twice.
  do
  {
    loopcnt++;

    //     scan all free ints.
    //     in some cases, a free int may be replaced with another one to be scanned next.
    k = 0;
    prvnumfree = numfree;
    numfree = 0;             // start list of ints still free after augmenting int reduction.
    while (k < prvnumfree)
    {
      i = free[k];
      k++;
    //       find minimum and second minimum reduced double over intumns.
      umin = assigndouble[i][0] - v[0];
      j1 = 0;
      usubmin = BIG;
      for (j = 1; j < dim; j++)
      {
        h = assigndouble[i][j] - v[j];
        if (h < usubmin)
          if (h >= umin)
          {
            usubmin = h;
            j2 = j;
          }
          else
          {
            usubmin = umin;
            umin = h;
            j2 = j1;
            j1 = j;
          }
      }

      i0 = csol[j1];
      if (umin < usubmin)
    //         change the reduction of the minimum intumn to increase the minimum
    //         reduced double in the int to the subminimum.
        v[j1] = v[j1] - (usubmin - umin);
      else                   // minimum and subminimum equal.
        if(i0 > -1)  // minimum intumn j1 is assigned.
        {
    //           swap intumns j1 and j2, as j2 may be unassigned.
          j1 = j2;
          i0 = csol[j2];
        }

    //       (re-)assign i to j1, possibly de-assigning an i0.
      rsol[i] = j1;
      csol[j1] = i;

        if(i0 > -1)  // minimum intumn j1 assigned earlier.
            if (umin < usubmin)
        //           put in current k, and go back to that k.
        //           continue augmenting path i - j1 with i0.
                free[--k] = i0;
            else
        //           no further augmenting reduction possible.
        //           store i0 in list of free ints for next phase.
              free[numfree++] = i0;
    }
  }
  while (loopcnt < 2);       // repeat once.

  // AUGMENT SOLUTION for each free int.
  for (f = 0; f < numfree; f++)
  {
    freeint = free[f];       // start int of augmenting path.

    // Dijkstra shortest path algorithm.
    // runs until unassigned intumn added to shortest path tree.
    for(j = dim;j--;)
    {
      d[j] = assigndouble[freeint][j] - v[j];
      pred[j] = freeint;
      intlist[j] = j;        // init intumn list.
    }

    low = 0; // intumns in 0..low-1 are ready, now none.
    up = 0;  // intumns in low..up-1 are to be scanned for current minimum, now none.
             // intumns in up..dim-1 are to be considered later to find new minimum,
             // at this stage the list simply contains all intumns
    unassignedfound = false;
    do
    {
      if (up == low)         // no more intumns to be scanned for current minimum.
      {
        last = low - 1;

        // scan intumns for up..dim-1 to find all indices for which new minimum occurs.
        // store these indices between low..up-1 (increasing up).
        min = d[intlist[up++]];
        for (k = up; k < dim; k++)
        {
          j = intlist[k];
          h = d[j];
          if (h <= min)
          {
            if (h < min)     // new minimum.
            {
              up = low;      // restart list at index low.
              min = h;
            }
            // new index with same minimum, put on undex up, and extend list.
            intlist[k] = intlist[up];
            intlist[up++] = j;
          }
        }
        // check if any of the minimum intumns happens to be unassigned.
        // if so, we have an augmenting path right away.
        for (k = low; k < up; k++)
          if (csol[intlist[k]] < 0)
          {
            endofpath = intlist[k];
            unassignedfound = true;
            break;
          }
      }

      if (!unassignedfound)
      {
        // update 'distances' between freeint and all unscanned intumns, via next scanned intumn.
        j1 = intlist[low];
        low++;
        i = csol[j1];
        h = assigndouble[i][j1] - v[j1] - min;

        for (k = up; k < dim; k++)
        {
          j = intlist[k];
          v2 = assigndouble[i][j] - v[j] - h;
          if (v2 < d[j])
          {
            pred[j] = i;
            if (v2 == min)   // new intumn found at same minimum value
              if (csol[j] < 0)
              {
                // if unassigned, shortest augmenting path is complete.
                endofpath = j;
                unassignedfound = true;
                break;
              }
              // else add to list to be scanned right away.
              else
              {
                intlist[k] = intlist[up];
                intlist[up++] = j;
              }
            d[j] = v2;
          }
        }
      }
    }
    while (!unassignedfound);

    // update intumn prices.
    for( k = last+1;k--;)
    {
      j1 = intlist[k];
      v[j1] = v[j1] + d[j1] - min;
    }

    // reset int and intumn assignments along the alternating path.
    do
    {
      i = pred[endofpath];
      csol[endofpath] = i;
      j1 = endofpath;
      endofpath = rsol[i];
      rsol[i] = j1;
    }
    while (i != freeint);
  }

  // calculate optimal double.
  double lapdouble = 0;
//  for (i = 0; i < dim; i++)
  for(i = dim;i--;)
  {
    j = rsol[i];
	u[i] = assigndouble[i][j] - v[j];
    lapdouble = lapdouble + assigndouble[i][j];
  }


  return lapdouble;
}


double lap_sparse(std::vector<std::tuple<int,int,double>>&costEdges,std::vector<int>&rowsol){
  
  operations_research::SimpleLinearSumAssignment assignment;
  for(auto const &edge:costEdges){
    assignment.AddArcWithCost(std::get<0>(edge),std::get<1>(edge),std::get<2>(edge));
  }
  if (assignment.Solve() == operations_research::SimpleLinearSumAssignment::OPTIMAL){
    for (int node = 0; node < assignment.NumNodes(); ++node){
      rowsol[node]=assignment.RightMate(node);
    } 
  }
  return assignment.OptimalCost();
}