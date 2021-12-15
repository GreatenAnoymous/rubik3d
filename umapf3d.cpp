/**
 * @file umapf3d.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-14
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "umapf3d.hpp"
#include <boost/graph/edmonds_karp_max_flow.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/read_dimacs.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/config.hpp>
#include <unordered_map>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>

// using namespace boost;
// using namespace std;
// using Traits=adjacency_list_traits<vecS, vecS, directedS>;
// using Graph= adjacency_list<listS, vecS,directedS,
// 	property < vertex_name_t, int,
// 	property < vertex_color_t, default_color_type,
// 	property < vertex_distance_t, long,
// 	property < vertex_predecessor_t, Traits::edge_descriptor >  > > >,
// 	property < edge_capacity_t, int,
// 	property < boost::edge_residual_capacity_t, int,
//     property < boost::edge_reverse_t, Traits::edge_descriptor> > >
// > ;


//     //the node in the network flow
//     struct flowNode{
//         Location3d u;
//         Location3d v;
//         double t;
//         Graph::vertex_descriptor vid;
//         flowNode(Location3d _u,Location3d _v,double _t){
//             u=_u;
//             v=_v;
//             t=_t;
//         }
//         flowNode(){};

//         bool operator==(const flowNode &other)const{
//             return this->u==other.u&&this->v==other.v&&this->t==other.t;
//         }
//     };
//     //hash function for flow node
//     namespace std{
//     template<>
//     struct hash<flowNode>{
//         size_t operator()(const flowNode&s)const{
//             size_t seed=0;
//             boost::hash_combine(seed,s.u.x);
//             boost::hash_combine(seed,s.u.y);
//             boost::hash_combine(seed,s.v.x);
//             boost::hash_combine(seed,s.v.y);
//             boost::hash_combine(seed,s.t);
//             return seed;
//         }
//     };
//     }

//     namespace std{
//     template<>
//     struct hash<Location3d>{
//         size_t operator()(const Location3d&s)const{
//             size_t seed=0;
//             boost::hash_combine(seed,s.x);
//             boost::hash_combine(seed,s.y);
//             boost::hash_combine(seed,s.x);
//             return seed;
//         }
//     };
//     }


//     //function to add an edge
//     void AddEdge(Traits::vertex_descriptor &v1,
//                     Traits::vertex_descriptor &v2,
//                     boost::property_map < Graph, boost::edge_reverse_t >::type &rev,
//                     const int capacity, 
//                     Graph &g);

//     Graph constructGraph(Configs &starts,Configs &goals,int xmax,int ymax/*,Configs &obstacles*/);

//     using capacityList= property_map < Graph, edge_capacity_t >::type ;
//     using residualList= property_map < Graph, edge_residual_capacity_t >::type ;
//     using NodeIdMap=std::unordered_map<flowNode,Graph::vertex_descriptor>;
//     using IdNodeMap=std::unordered_map<Graph::vertex_descriptor,flowNode>;

//     Paths3d retrieve_paths(capacityList&capacity,residualList &residual_capacity,Graph &graphx,Configs &starts,
//         std::unordered_map<Graph::vertex_descriptor,flowNode> &id_node,
//         std::unordered_map<flowNode,Graph::vertex_descriptor>& node_id,Graph::vertex_descriptor &sink_id,
//         Configs &goals);

//     void AddEdge(	Traits::vertex_descriptor &v1,
//                     Traits::vertex_descriptor &v2,
//                     boost::property_map < Graph, boost::edge_reverse_t >::type &rev,
//                     const int capacity, 
//                     Graph &g)
//     {
//         Traits::edge_descriptor e1 = boost::add_edge(v1, v2, g).first;
//         Traits::edge_descriptor e2 = boost::add_edge(v2, v1, g).first;
//         boost::put(boost::edge_capacity, g, e1, capacity);
//         boost::put(boost::edge_capacity, g, e2, capacity);
//         rev[e1] = e2;
//         rev[e2] = e1;
        
//     }

//     Graph::vertex_descriptor insert_node(IdNodeMap&id_node,
//                     NodeIdMap &node_id,
//                     flowNode &node,int id,Graph &g){
//         if(node_id.find(node)==node_id.end()){
//             Graph::vertex_descriptor v=add_vertex(g);
//             node.vid=v;
//             node_id.insert({node,v});
//             id_node.insert({v,node});
//             return v;
//             // id=id+1;
//         }
//         else{
//             Graph::vertex_descriptor v=node_id[node];
//             node.vid=v;
//             return v;
//         }
        
//     }



//     void constructGraph(Configs &starts, Configs &goals, int xmax, int ymax, int zmax,/*Obstacles &obstacles,*/int timeSteps,
//         Graph &graphx,IdNodeMap &id_node,
//         NodeIdMap& node_id,
//         Graph::vertex_descriptor &source_id,Graph::vertex_descriptor &sink_id)
//     {
//         std::unordered_set<Location3d> reachable_vertices;
//         boost::property_map < Graph, boost::edge_reverse_t >::type rev = get(boost::edge_reverse, graphx);
//         for(int r=0;r<starts.size();r++){
//             reachable_vertices.insert(*starts[r]);
//         }
//         int id=0;
//         for(int t=0;t<timeSteps;t++){
//             std::unordered_set<Location3d> next_reachable_vertices;
//             for(auto v1:reachable_vertices){
//                 std::vector<Location3d> neighbors={
//                     Location3d(-1, v1.x+1, v1.y, v1.z), Location3d(-1,v1.x,v1.y+1,v1.z), Location3d(-1, v1.x, v1.y, v1.z+1),
//                     Location3d(-1, v1.x-1, v1.y, v1.z), Location3d(-1,v1.x,v1.y-1,v1.z), Location3d(-1,v1.x,v1.y,v1.z-1)};
//                 flowNode node1(v1,v1,t);
//                 for(auto v2:neighbors){
//                     if(v2.x<0||v2.y<0||v2.x>=xmax||v2.y>=ymax||v2.z<0||v2.z>=zmax) continue;
//                     // if(obstacles.find(v2)!=obstacles.end()) continue;
                
//                     insert_node(id_node,node_id,node1,id,graphx);
//                     std::vector<Location3d> edge_node;
//                     if(v1<v2) edge_node={v1,v2};
//                     else edge_node={v2,v1};
//                     flowNode node2(edge_node[0],edge_node[1],t+0.3);
//                     insert_node(id_node,node_id,node2,id,graphx);
//                     flowNode node3(edge_node[0],edge_node[1],t+0.6);
//                     insert_node(id_node,node_id,node3,id,graphx);
//                     flowNode node4(v2,v2,t+0.9);
//                     insert_node(id_node,node_id,node4,id,graphx);
//                     // std::cout<<id_node.size()<<" -----"<<node_id.size()<<std::endl;
//                     AddEdge(node1.vid,node2.vid,rev,1,graphx);
//                     AddEdge(node2.vid,node3.vid,rev,1,graphx);
//                     AddEdge(node3.vid,node4.vid,rev,1,graphx);
//                     next_reachable_vertices.insert(v2);
//                 }
//                 flowNode node5(v1,v1,t+0.9);
//                 flowNode node6(v1,v1,t+1);

//                 insert_node(id_node,node_id,node5,id,graphx);
//                 insert_node(id_node,node_id,node6,id,graphx);
//                 AddEdge(node5.vid,node6.vid,rev,1,graphx);
//                 AddEdge(node1.vid,node6.vid,rev,1,graphx);
//                 next_reachable_vertices.insert(v1);
//             }
//             reachable_vertices=next_reachable_vertices;   
            
//         }
//         //add source and sinks
//         flowNode source(Location3d(-1, 0,0,0),Location3d(-1, xmax,xmax,xmax),0);
//         flowNode sink(Location3d(-1, ymax,ymax,ymax),Location3d(-1, 0,0,0),0);
//         source_id=insert_node(id_node,node_id,source,id,graphx);
//         sink_id=insert_node(id_node,node_id,sink,id,graphx);
    
//         for(int i=0;i<starts.size();i++){
//             Graph::vertex_descriptor s_id=node_id[flowNode(*starts[i],*starts[i],0)];
//             // Graph::vertex_descriptor vs=id_node[s_id];
//             AddEdge(source_id,s_id,rev,1,graphx);
//             Graph::vertex_descriptor g_id=node_id[flowNode(*goals[i],*goals[i],timeSteps)];
//             // Graph::vertex_descriptor vg=id_node[g_id].vid;
//             AddEdge(g_id,sink_id,rev,1,graphx);
//         }
//     }




//     Paths3d retrieve_paths(capacityList&capacity,residualList &residual_capacity,Graph &graphx,Configs &starts,
//         std::unordered_map<Graph::vertex_descriptor,flowNode> &id_node,
//         std::unordered_map<flowNode,Graph::vertex_descriptor>& node_id,Graph::vertex_descriptor &sink_id,
//         Configs &goals){
//         graph_traits < Graph >::vertex_iterator u_iter, u_end;
//         graph_traits < Graph >::out_edge_iterator ei, e_end;
//         std::unordered_map<Graph::vertex_descriptor,Graph::vertex_descriptor> adj_list;
//         for (boost::tie(u_iter, u_end) = vertices(graphx); u_iter != u_end; ++u_iter){
//             for (boost::tie(ei, e_end) = out_edges(*u_iter, graphx); ei != e_end; ++ei){
//                 if (capacity[*ei] > 0&&(capacity[*ei]-residual_capacity[*ei])==1){
//                     auto head_i=*u_iter ;
//                     auto tail_i=boost::target(*ei,graphx);
            
//                     adj_list[head_i]=tail_i;
//                 }
//             }
//         }
//         Paths3d paths;
//         for(int i=0;i<starts.size();i++){
//             Path3d path;
//             flowNode start_node(*starts[i],*starts[i],0);
//             auto current_id=node_id[start_node];
//             while(current_id!=sink_id){
//                 double timestep=id_node[current_id].t;
//                 if(fabs(timestep-int(timestep))<1e-2){
//                     auto node= new Location3d(id_node[current_id].u);
//                     path.push_back(node);
//                 }
//                 current_id=adj_list[current_id];
//             }
//             paths.push_back(path);
//             // std::cout<<"found the paths"<<std::endl;
//         }
//         // std::cout<<paths.size()<<" "<<goals.size()<<std::endl;
//         for(int i=0;i<goals.size();i++){
//             goals[i]=paths[i][paths[i].size()-1];
//         }
//         return paths;

//     }



void UMAPF3d::solve(){
    // using namespace umapf3d;
    // using namespace std;
    // int timeSteps = 4;
    // while(true){
    //     try{
    //         std::unordered_map<Graph::vertex_descriptor,flowNode> id_node;
    //         std::unordered_map<flowNode,Graph::vertex_descriptor> node_id;
    //         Graph graphx;
    //         Graph::vertex_descriptor source,sink;
    //         constructGraph(starts,goals,graph->xmax,graph->ymax,graph->zmax,/*obstacles,*/timeSteps,graphx,id_node,node_id,source,sink);
    //         //std::cout<<"source id=?"<<source<<" sink_id=?"<<sink<<"  map size="<<id_node.size()<<std::endl;
    //         capacityList capacity = get(edge_capacity, graphx);
    //         //property_map < Graph, edge_reverse_t >::type rev = get(edge_reverse, g);
    //         residualList residual_capacity = get(edge_residual_capacity, graphx);
    //         // int flow=edmonds_karp_max_flow(graphx,source,sink);
    //         int flow=boykov_kolmogorov_max_flow(graphx,source,sink);
            
    //         if(flow==starts.size()){
    //             Paths3d paths=retrieve_paths(capacity,residual_capacity,graphx,starts,id_node,node_id,sink,goals);
    //             result = paths;
    //             return;
    //         }
    //         else{
    //             timeSteps++;
    //         }
    //     }catch(std::runtime_error &e){
    //         std::cout << "Exception: " << std::endl;
    //     }
    // }
    // result = ;
}
