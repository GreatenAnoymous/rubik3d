/**
 * @file RTH3d_obs.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-01-30
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "RTH3d_obs.hpp"
#include "formation.hpp"
#include <thread>
#include "mp3d.hpp"

void RTH_3d_obs::random_to_balanced_fast()
{
    Configs bl_config1;
    for (int i = 0; i < graph->xmax; i += 3)
    {
        for (int j = 0; j < graph->ymax; j += 3)
        {
            for (int k = 0; k < graph->zmax; k += 3)
            {
                Configs tmp_list = {getVertex(i + 1, j, k), getVertex(i + 1, j + 2, k),
                                    getVertex(i + 1, j, k + 1), getVertex(i + 1, j + 2, k + 1),
                                    getVertex(i + 1, j, k + 2), getVertex(i + 1, j + 2, k + 2)};
                bl_config1.insert(bl_config1.end(), tmp_list.begin(), tmp_list.end());
            }
        }
    }
    Configs starts, goals;
    for (auto agent : robots)
    {
        starts.push_back(agent->start);
        goals.push_back(agent->goal);
    }

    auto umapf = [&](Configs &starts, Configs &goals, Paths3d &returned)
    {
        FormationControl solver(starts, goals, graph);
        solver.solve();
        returned = solver.get_result();
    };

    Paths3d paths_s, paths_g;

    std::thread th1(umapf, std::ref(starts), std::ref(bl_config1), std::ref(paths_s));
    std::thread th2(umapf, std::ref(goals), std::ref(bl_config1), std::ref(paths_g));
    th1.join();
    th2.join();
    format_paths(paths_s);
    // exit(0);
    format_paths(paths_g);

    // print_one_path(paths_s[0]);
    // print_one_path(paths_g[0]);
    // printf("(%d,%d,%d)\n",starts[0]->x,starts[0]->y,starts[0]->z);
    // exit(0);
    for (int i = 0; i < robots.size(); i++)
    {
        // agents[i]->start=paths_s[i].back();
        // agents[i]->goal=paths_g[i].back();
        robots[i]->current = paths_s[i].back();
        robots[i]->umapf_goal = paths_g[i].back();
        robots[i]->path.insert(robots[i]->path.end(), paths_s[i].begin() + 1, paths_s[i].end());
        robots[i]->umapf_goal_path = paths_g[i];
        std::reverse(robots[i]->umapf_goal_path.begin(), robots[i]->umapf_goal_path.end());
    }
}

void RTH_3d_obs::z_shuffle()
{
    for (int i = 0; i < graph->xmax; i += cell_size)
    {
        for (int j = 0; j < graph->ymax; j++)
        {
            Robots agents;
            for (auto &r : robots)
            {

                if (r->current->x < i + cell_size and r->current->x >= i and r->current->y == j)
                    agents.push_back(r);
            }
            if (agents.empty() == true)
                continue;
            if (j % cell_size == 1)
                continue;
            // std::cout << " debug robots size=" << agents.size() << std::endl;
            Motion3d swapper(agents, {i, i + cell_size - 1}, {j, j}, {0, graph->zmax - 1}, 'z', graph);
            swapper.reconfigure();
        }
    }

    fill_paths(robots);
}

void RTH_3d_obs::solve()
{
    std::cout << "num of agents with obstacles=" << robots.size() << std::endl;
    random_to_balanced_fast();
    // first do fat column (xz-plane) matching
    lba_matching_fat();
    z_shuffle();
    // for each x-z plane apply 3-m shuffle to get to the matched positions
    xy_fitting();
    xy_shuffle();

    z_fitting();
    z_shuffle();

    append_umapf_path();
}

void RTH_3d_obs::lba_matching_fat()
{
    using point2d = std::pair<int, int>; // x,y

    // std::unordered_map<point2d,Robots,boost::hash<point2d>> fat_column_dict;
    std::unordered_map<int, Robots> fat_column_dict;

    for (auto &r : robots)
    {
        // point2d key={r->current->x,r->current->y};
        int key = get_plane_id(r->current->x, r->current->y);
        fat_column_dict[key].push_back(r);
    }

    for (int i = 0; i < graph->zmax; i++)
    {
        std::unordered_map<int, int> matching;
        std::unordered_map<point2d, Robot *, boost::hash<point2d>> arranged_robots;
        matching_helper(fat_column_dict, matching, arranged_robots, i);
        // std::set<int> key_set, col_set;
        // for (auto [key, col] : matching)
        // {
        //     key_set.insert(key);
        //     col_set.insert(col);
        //     std::cout << key << " " << col << std::endl;
        // }
        // std::cout << "debug key col equal? " << (key_set == col_set) << std::endl;
        // std::cout << "aranged robots size=" << arranged_robots.size() << std::endl;

        for (auto &[id, robot] : arranged_robots)
        {
            int xi = robot->current->x;
            int yi = robot->current->y;
            int key = get_plane_id(xi, yi);
            int column = matching[key];
            robot->intermediate = graph->getVertex(xi, yi, i);
        
        }
        // for(int xi=1;xi<graph->xmax;xi+=cell_size){
        //     for(int yi=0;yi<graph->ymax;yi++){
        //         if(yi%cell_size==1) continue;
        //         int key=get_plane_id(xi,yi);
        //         int column=matching[key];

        //         // assert(arranged_robots.find({key,column})!=arranged_robots.end());
        //         arranged_robots[{key,column}]->intermediate=
        //             graph->getVertex(xi,yi,i);
        //         // std::cout<<"robot "<<arranged_robots[{key,column}]->id<<" aranged in ("<<xt<<","<<yt<<","<<i<<")\n";
        //     }
        // }

        // for (auto &robot : robots)
        // {
        //     assert(robot->intermediate != nullptr);
        //     assert(robot->intermediate->x == robot->current->x);
        //     assert(robot->intermediate->y == robot->current->y);
        //     // std::cout<<robot->intermediate->print()<<"================"<<std::endl;
        // }
    }
    // LBA_heuristic();
}