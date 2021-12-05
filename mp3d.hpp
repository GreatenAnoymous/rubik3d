/**
 * @file mp3d.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once

#include"common.hpp"

class Motion3d{
public:
    using point2d=std::pair<int,int>;
    Motion3d(){}
    Motion3d(Robots &robots,point2d xrange,point2d yrange,point2d zrange,char orientation='x',Grids3d *graph=nullptr);
    void reconfigure();
    std::function<Location3d*(int ,int,int)> getVertex;
protected:
    int xmin,xmax,ymin,ymax,zmin,zmax;
    Robots robots;
    void prepare();
    void prepare_helper(int lx,int ly,int lz);
    void reconfigure_x();
    void reconfigure_y();
    void reconfigure_z();
    void insert_end_path();
    Grids3d* graph;
    char orientation;
};