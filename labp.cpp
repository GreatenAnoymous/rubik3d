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

