/**
 * @file mp3d.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include"mp3d.hpp"




///////////////////////////////motion3d/////////////////////////////////////////////////
Motion3d::Motion3d(Robots &robots,point2d xrange,point2d yrange,point2d zrange,char orientation){
    this->robots=robots;
    this->xmin=xrange.first;
    this->xmax=xrange.second;
    this->ymin=yrange.first;
    this->ymax=yrange.second;
    this->zmin=zrange.first;
    this->zmax=zrange.second;
    this->orientation=orientation;
}


void Motion3d::reconfigure(){
    switch (orientation){
        case 'x':
            reconfigure_x();
            break;
        case 'y':
            reconfigure_y();
            break;
        case 'z':
            reconfigure_z();
            break;
    }
}

/**
 * @brief 
 * mvoe all the robots to the middle line
 */
void Motion3d::prepare(){
    const int cell_size=3;
    switch(orientation){
        case 'x':
            for(int i=xmin;i<xmax+1;i+=cell_size) prepare_helper(i,ymin,zmin);break;
        case 'y':
            for(int i=ymin;i<ymax+1;i+=cell_size) prepare_helper(xmin,i,zmin);break;
        case 'z':
            for(int i=zmin;i<zmax+1;i+=cell_size) prepare_helper(xmin,ymin,i);break;
    }
    fill_paths(robots);
}

/**
 * @brief 
 * shuffle x from 1 to m
 */

void Motion3d::reconfigure_x(){
    if(robots.size()==0) return;
    prepare();
    fill_paths(robots);
    for(auto &r:robots){
        int xs,ys,zs,xg,yg,zg;
        std::tie(xs,ys,zs)=std::make_tuple(r->current->x,r->current->y,r->current->z);
        std::tie(xg,yg,zg)=std::make_tuple(r->inter2->x,r->inter2->y,r->inter2->z);
        if(xg>xs){
            for(int x=xs;x<xg+1;x++){
                r->path.emplace_back(getVertex(x,ys+1,zs));
            }
        }
        else{
            for(int x=xs;x>xg-1;x--){
                r->path.emplace_back(getVertex(x,ys-1,zs));
            }
        }
        r->path.emplace_back(getVertex(xg,yg,zg));
    }
    fill_paths(robots);
    insert_end_path();
    fill_paths(robots);
}

void Motion3d::reconfigure_y(){
    if(robots.empty()) return;
    prepare();
    fill_paths(robots);
    for(auto &r:robots){
        int xs,ys,zs,xg,yg,zg;
        std::tie(xs,ys,zs)=std::make_tuple(r->current->x,r->current->y,r->current->z);
        std::tie(xg,yg,zg)=std::make_tuple(r->inter2->x,r->inter2->y,r->inter2->z);
        if(yg>ys){
            for(int y=ys;y<yg+1;y++)
                r->path.emplace_back(getVertex(xs+1,y,zs));
        }
        else{
            for(int y=ys;y>yg-1;y--)
                r->path.emplace_back(getVertex(xs-1,y,zs));
        }
        r->path.emplace_back(getVertex(xg,yg,zg));
    }
    fill_paths(robots);
    insert_end_path();
    fill_paths(robots);
}

void Motion3d::reconfigure_z(){
    if(robots.empty()) return;
    prepare();
    fill_paths(robots);
    for(auto &r:robots){
        int xs,ys,zs,xg,yg,zg;
        std::tie(xs,ys,zs)=std::make_tuple(r->current->x,r->current->y,r->current->z);
        std::tie(xg,yg,zg)=std::make_tuple(r->inter2->x,r->inter2->y,r->inter2->z);
        if(zg>zs){
            for(int z=zs;z<zg+1;z++)
                r->path.emplace_back(getVertex(xs,ys+1,z));
        }
        else{
            for(int z=zs;z>zg-1;z--)
                r->path.emplace_back(getVertex(xs,ys-1,z));
        }
        r->path.emplace_back(getVertex(xg,yg,zg));
    }
    fill_paths(robots);
    insert_end_path();
    fill_paths(robots);
}


void Motion3d::prepare_helper(int,int,int){

}

void Motion3d::insert_end_path(){
    
}