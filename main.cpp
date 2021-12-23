/**
 * @file main.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */


#include "common.hpp"
#include "mp3d.hpp"
#include "rth2d.hpp"
#include "rubik3d.hpp"
#include "umapf3d.hpp"
#include "json.hpp"
#include "formation.hpp"

nlohmann::json data2d; 

void test_read_instance(){
    Robots test_robots;
    Grids3d *test_graph;
    read_instances("./test.scen",test_robots,test_graph);
    // std::cout<<"what is wrong "<<test_robots.size()<<" ";
    std::cout<<test_graph->getNodesSize()<<" "<<test_graph->xmax<<std::endl;
}


void test_umapf_3d(){
   
}


void test_rth2d(){
    std::ifstream ifs("./data2d.json");
    data2d=nlohmann::json::parse(ifs);
    // std::cout<<data2d.size()<<std::endl;
    Robots test_robots;
    Grids3d*test_graph;
    
    read_instances("./python/debug2d.scen",test_robots,test_graph);
    std::cout<<"debug size "<<test_graph->getNodesSize()<<std::endl;
    RTH_2d tester(test_robots,test_graph);
    tester.solve();
}


void test_rth3d(){
    std::ifstream ifs("./data2d.json");
    data2d=nlohmann::json::parse(ifs);
    Robots test_robots;
    Grids3d *test_graph;
    read_instances("./python/instances/quasi_random/9x9_1.scen",test_robots,test_graph);
    RTH_3d solver(test_robots,test_graph);
    solver.solve();
    solver.save_result("./test.txt",0,false);
    
    
}

void test_formation3d(){
    Robots test_robots;
    Grids3d *test_graph;
    // read_instances("./debug.scen",test_robots,test_graph);
    read_instances("./python/instances/quasi_random/9x9_10.scen",test_robots,test_graph);
    std::cout<<"num agents="<<test_robots.size()<<std::endl;
    FormationControl test(test_robots,test_graph);
    test.solve();

}


void rth3dexe(int argc,char* argv[]){
    std::ifstream ifs("./data2d.json");
    data2d=nlohmann::json::parse(ifs);
    std::string scen_file=argv[1];
    std::string out_file=argv[2];
    Robots test_robots;
    Grids3d *test_graph;
    read_instances(scen_file,test_robots,test_graph);
    RTH_3d solver(test_robots,test_graph);
    auto t1=Time::now();
    solver.solve();
    auto t2=Time::now();
    fsec dt=t2-t1;
    solver.save_result(out_file,dt.count(),false);

}

int main(int argc, char* argv[]){
    // test_read_instance();
    // test_rth2d();
    // test_formation3d();
    // test_rth3d();
    rth3dexe(argc,argv);
    return 0;

}