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

}


void test_rth3d(){

}


int main(int argc, char* argv[]){
    test_read_instance();
    return 0;

}