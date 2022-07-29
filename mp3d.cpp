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
Motion3d::Motion3d(Robots &robots,point2d xrange,point2d yrange,point2d zrange,char orientation,Grids3d *_graph){
    this->robots=robots;
    this->xmin=xrange.first;
    this->xmax=xrange.second;
    this->ymin=yrange.first;
    this->ymax=yrange.second;
    this->zmin=zrange.first;
    this->zmax=zrange.second;
    this->orientation=orientation;
    this->graph=_graph;
    getVertex=[&](int x,int y,int z){
        return graph->getVertex(x,y,z);
    };
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
    // const int cell_size=3;
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
 * shuffle x from 1 to m, using xy
 */
void Motion3d::reconfigure_x(){
    if(robots.size()==0) return;
    prepare();
    // std::cout<<"the size="<<graph->getNodesSize()<<std::endl;
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

/**
 * @brief  using xy
 * 
 */
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

/**
 * @brief using xz
 * 
 */
void Motion3d::reconfigure_z(){

    if(robots.empty()) return;
    // for(auto &robot:robots){
    //     std::cout<<robot->current->print()<<"---->"<<robot->intermediate->print()<<std::endl;
    // }
    // printf("(%d,%d),(%d,%d),(%d,%d),|r|=%d\n",xmin,xmax,ymin,ymax,zmin,zmax,(int)robots.size());
    // std::cout<<"debug dimension "<<xmax<<" "<<ymax<<" "<<zmax<<" |r|="<<robots.size()<<std::endl;
    prepare();
    
    fill_paths(robots);
  
    for(auto &r:robots){
        int xs,ys,zs,xg,yg,zg;
        std::tie(xs,ys,zs)=std::make_tuple(r->current->x,r->current->y,r->current->z);
        assert(r->inter2!=nullptr);
    
        
        std::tie(xg,yg,zg)=std::make_tuple(r->inter2->x,r->inter2->y,r->inter2->z);
        // printf("(%d,%d,%d)--->(%d,%d,%d)\n",xs,ys,zs,xg,yg,zg);
        if(zg>zs){
            for(int z=zs;z<zg+1;z++)
                r->path.emplace_back(getVertex(xs+1,ys,z));
        }
        else{
            for(int z=zs;z>zg-1;z--)
                r->path.emplace_back(getVertex(xs-1,ys,z));
        }
        //  std::cout<<r->id<<std::endl;
        // for(auto &v:r->path) printf("(%d %d %d)=>",v->x,v->y,v->z);
        // std::cout<<"\n";
        r->path.emplace_back(getVertex(xg,yg,zg));
       
    }

    
  
    fill_paths(robots);
    insert_end_path();
    fill_paths(robots);
}




void Motion3d::insert_end_path(){
    for(auto &agent:robots) {
        agent->path.insert(agent->path.end(), agent->inter_path.begin(), agent->inter_path.end());
        agent->current = agent->path.back();
    }
}
/**
 * @brief 
 * 
 * @param min_x 
 * @param min_y 
 * @param min_z 
 */

void Motion3d::prepare_helper(int min_x,int min_y,int min_z){
    // std::cout<<"min_x="<<min_x<<std::endl;
    std::function<int(int,int,int)> get_json_id;
    std::function<std::string(Robots &,char)>get_json_key;
    std::function<Location3d *(int vid)> get_back_vertex;
    Robots robots1,robots2;
    std::vector<Location3d*>vertices;
 
    // for(auto &robot:robots){
    //     // printf("robot %d: (%d,%d,%d) \n",robot->id,robot->intermediate->x,robot->intermediate->y,robot->intermediate->z);
    //     // std::cout<<"robot ";
    //     // std::cout<<robot->id<<" :";
    //     std::cout<<robot->start->print()<<" "<<robot->intermediate->print()<<std::endl;
    // }

    auto get_index = [](std::vector<Location3d*> &vertices, Location3d *v) {
        auto it = find(vertices.begin(), vertices.end(), v);
        int id = distance(vertices.begin(), it);
        return id;
    };

    get_json_key=[&vertices,&get_index](Robots &r,char sg)->std::string{
        std::string start_id="((";
        std::function<bool(Robot*,Robot*)> id_order;
        if(sg=='s') id_order=[&vertices,&get_index](Robot *r1,Robot *r2){
            int id1=get_index(vertices,r1->current);
            int id2=get_index(vertices,r2->current);
            return id1<id2;
        };
        else if(sg=='g')id_order=[&vertices,&get_index](Robot *r1,Robot *r2){
            int id1=get_index(vertices,r1->intermediate);
            int id2=get_index(vertices,r2->intermediate);
            return id1<id2;
        };
        std::sort(r.begin(),r.end(),id_order);
        for(auto &ri:r){
            int id;
            if(sg=='s')id=get_index(vertices,ri->current);
            else id=get_index(vertices,ri->intermediate);
            start_id+=std::to_string(id);
            start_id+=", ";
        }
        start_id.pop_back();
        start_id.pop_back();
        start_id+="), ";
        start_id += "'x')";
        return start_id;
    };
    
    
    auto find_robots=[&](std::vector<Location3d*>&vertices,Robots &r1,Robots &r2){
        for (auto &agent : robots){
            if (std::find(vertices.begin(), vertices.end(), agent->current) != vertices.end()){
                r1.push_back(agent);
            }
            if (std::find(vertices.begin(), vertices.end(), agent->intermediate) != vertices.end()){
                r2.push_back(agent);
            }
        }
    };

    if(orientation=='x'){
        for(int y=min_y;y<min_y+cell_size;y++)
            for(int x=min_x;x<min_x+cell_size;x++)
                vertices.push_back(getVertex(x,y,min_z));
  
        find_robots(vertices,robots1,robots2);
       
    }

    else if(orientation=='y'){
        for(int x=min_x;x<min_x+cell_size;x++)
            for(int y=min_y;y<min_y+cell_size;y++)
                vertices.push_back(getVertex(x,y,min_z));

        find_robots(vertices,robots1,robots2);

    }

    else if(orientation=='z'){
    
        for(int x=min_x;x<min_x+cell_size;x++)
            for(int z=min_z;z<min_z+cell_size;z++){
                // printf("debug %d %d %d\n",x,min_y,z);
                vertices.push_back(getVertex(x,min_y,z));
            }
                

        find_robots(vertices,robots1,robots2);
    }
    // std::cout<<" robots1 size="<<robots1.size()<<" robots2 size="<<robots2.size()<<std::endl;
    if(robots1.size()!=0){
        auto start_id=get_json_key(robots1,'s');
      
        // std::cout<<data2d.size()<<std::endl;
        std::vector<std::vector<int>> solution = data2d[start_id];
        for(int i=0;i<robots1.size();i++){
            Path3d new_path;
            for (auto vid : solution[i]){
                new_path.push_back(vertices[vid]);
            }
            // assert(new_path[0]==robots2[i]->intermediate);
             robots1[i]->path.insert(robots1[i]->path.end(), new_path.begin(), new_path.end());
            robots1[i]->current = new_path.back();
        }
    }
    if(robots2.size()!=0){
        
        auto start_id=get_json_key(robots2,'g');
        // std::cout<<start_id<<std::endl;
        std::vector<std::vector<int>> solution = data2d[start_id];
        for(int i=0;i<robots2.size();i++){
            Path3d new_path;
            for (auto vid : solution[i]){
                new_path.push_back(vertices[vid]);
            }
            assert(new_path[0]==robots2[i]->intermediate);
            robots2[i]->inter2 = new_path.back();
          
            new_path.pop_back();
            std::reverse(new_path.begin(), new_path.end());
            robots2[i]->inter_path = new_path;
        }
    }


    
}
