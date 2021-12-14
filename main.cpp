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
    Robots test_robots;
    Grids3d *test_graph;
    read_instances("./test.scen",test_robots,test_graph);
}

void test_formation3d(){
    Robots test_robots;
    Grids3d *test_graph;
    read_instances("./debug.scen",test_robots,test_graph);
    std::cout<<"num agents="<<test_robots.size()<<std::endl;
    FormationControl test(test_robots,test_graph);
    test.solve();

}


int main(int argc, char* argv[]){
    // test_read_instance();
    // test_rth2d();
    test_formation3d();
    return 0;

}