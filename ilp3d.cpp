/**
 * @file ilp3d.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2021-12-29
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "ilp3d.hpp"
#include <thread>
#include "search.hpp"

Ilp3D::Ilp3D(Configs &starts, Configs &goals, Grids3d *graph) : starts(starts), goals(goals), graph(graph)
{
}

void Ilp3D::solve_original()
{
    int timeStep = 0;
    int numAgemts = starts.size();
    for (int i = 0; i < numAgemts; i++)
    {
        timeStep = std::max(starts[i]->manhattan_dist(goals[i]) - 1, timeStep);
    }
    while (true)
    {
        GRBModel model = prepare_model(timeStep);
        std::cout<<"model prepared! timestep="<<timeStep<<std::endl;
        model.optimize();
        double obj_val = model.get(GRB_DoubleAttr_ObjVal);
        std::cout<<"obj_val="<<obj_val<<std::endl;
        if (obj_val == numAgemts)
        {
            // retrive_paths(model, timeStep);
            makespan=timeStep;
            return;
        }
        timeStep += 1;
    }
}

GRBModel Ilp3D::prepare_model(int timeStep)
{
    bool relaxed = false;
    int numAgents = starts.size();
    edge_var_map = std::map<id_type, GRBVar>();
    edge_time_vector_map = std::map<id_type, std::vector<GRBVar>>();
    time_in_vector_map = std::map<id_type, std::vector<GRBVar>>();
    time_out_vector_map = std::map<id_type, std::vector<GRBVar>>();
    robot_in_vector_map = std::map<id_type, std::vector<GRBVar>>();
    robot_out_vector_map = std::map<id_type, std::vector<GRBVar>>();

    GRBModel model = GRBModel(*envp);
    model.set(GRB_IntParam_OutputFlag, 0);
    // Set variables
    std::set<Location3d *> reachable_vertices;
    std::set<Location3d *> new_vertices;

    for (size_t r = 0; r < numAgents; r++)
    {
        reachable_vertices = std::set<Location3d *>();
        reachable_vertices.insert(starts[r]);
        for (size_t t = 1; t < timeStep + 1; t++)
        {
            new_vertices = std::set<Location3d *>();
            for (auto n : reachable_vertices)
            {
                auto nbrs = graph->getNeighbors(n);
                nbrs.push_back(n);
                for (auto nbr : nbrs)
                {
                    // Check goal reachability
                    if (nbr->manhattan_dist(goals[r]) > timeStep - t)
                        continue;
                    new_vertices.insert(nbr);
                    id_type id = get_id(r, n, nbr, t - 1);
                    GRBVar x = model.addVar(0.0, 1.0, 0.0, relaxed ? GRB_CONTINUOUS : GRB_BINARY, id);
                    edge_var_map.insert(std::make_pair(id, x));
                    store_var_for_vertices(x, n, nbr, t - 1, t);
                    store_var_for_robots(x, r, n, nbr, t - 1, t);
                    store_var_for_edges(x, n, nbr, t - 1);
                }
                // Check goal reachability
                if (n->manhattan_dist(goals[r]) > timeStep - t)
                    continue;
                new_vertices.insert(n);
                id_type id = get_id(r, n, n, t - 1);
                GRBVar x = model.addVar(0.0, 1.0, 0.0, relaxed ? GRB_CONTINUOUS : GRB_BINARY, id);
                edge_var_map.insert(std::make_pair(id, x));
                store_var_for_vertices(x, n, n, t - 1, t);
                store_var_for_robots(x, r, n, n, t - 1, t);
            }
            reachable_vertices = new_vertices;
        }
    }
    // Set objective
    GRBLinExpr obj_expr = GRBLinExpr();
    for (size_t r = 0; r < numAgents; r++)
    {
        id_type id = get_id(r, goals[r], starts[r], timeStep);
        GRBVar x;
        x = model.addVar(0.0, 1.0, 0.0, relaxed ? GRB_CONTINUOUS : GRB_BINARY, id);
        edge_var_map.insert(std::make_pair(id, x));
        store_var_for_vertices(x, goals[r], starts[r], timeStep, 0);
        store_var_for_robots(x, r, goals[r], starts[r], timeStep, 0);
        obj_expr += x;
        // model.addConstr(x, GRB_EQUAL, 1);
    }
    model.setObjective(obj_expr, GRB_MAXIMIZE);
    // Set constraints
    for (size_t t = 0; t <= timeStep; t++)
        for (size_t v = 0; v < graph->getNodesSize(); v++)
        {
            auto it = time_in_vector_map.find(get_id(99999, graph->getVertex(v), nullptr, t));
            if (it == time_in_vector_map.end())
                continue;
            GRBLinExpr expr = GRBLinExpr();
            for (size_t i = 0; i < it->second.size(); i++)
                expr += it->second[i];
            if (expr.size() > 1)
                model.addConstr(expr, GRB_LESS_EQUAL, 1);
        }
    for (size_t r = 0; r < numAgents; r++)
    {
        for (size_t t = 0; t <= timeStep; t++)
        {
            for (size_t v = 0; v < graph->getNodesSize(); v++)
            {
                GRBLinExpr expr = GRBLinExpr();
                auto it = robot_in_vector_map.find(get_id(r, graph->getVertex(v), nullptr, t));
                if (it != robot_in_vector_map.end())
                    for (size_t i = 0; i < it->second.size(); i++)
                        expr -= it->second[i];
                it = robot_out_vector_map.find(get_id(r, graph->getVertex(v), nullptr, t));
                if (it != robot_out_vector_map.end())
                    for (size_t i = 0; i < it->second.size(); i++)
                        expr += it->second[i];
                if (expr.size() > 0)
                    model.addConstr(expr, GRB_EQUAL, 0);
            }
        }
    }
    for (auto it = edge_time_vector_map.begin(); it != edge_time_vector_map.end(); it++)
    {
        GRBLinExpr expr = GRBLinExpr();
        for (size_t i = 0; i < it->second.size(); i++)
            expr += it->second[i];
        if (expr.size() > 1)
            model.addConstr(expr, GRB_LESS_EQUAL, 1);
    }
    // model.write("m.lp");
    model.update();
    return model;
}

void Ilp3D::retrive_paths(GRBModel & model, int time_steps){
    auto vars = model.getVars();
    int numAgents=starts.size();
    final_paths = Paths3d(numAgents, Path3d(time_steps + 1, nullptr));
    for (size_t r = 0; r <numAgents; r++)
    {
        final_paths[r][0] = starts[r];
        for (size_t t = 1; t < time_steps + 1; t++)
        {
            Location3d* v1 = final_paths[r][t - 1];
            // assert(v1!=nullptr);
            std::vector<Location3d*> nbrs = graph->getNeighbors(v1);
            nbrs.push_back(v1);
            for (size_t i = 0; i < nbrs.size(); i++)
            {
                try
                {
                    if (model.getVarByName(get_id(r, v1, nbrs[i], t - 1)).get(GRB_DoubleAttr_X) > 1.0)
                    {
                        // assert(nbrs[i]!=nullptr);
                        final_paths[r][t] = nbrs[i];
                        break;
                    }
                }
                catch (const GRBException &)
                {
                    continue;
                }
            }
            // if(final_paths[r][t]==nullptr) {
            //     for(size_t i = 0; i < nbrs.size(); i++){
            //         std::cout<<"var value="<<model.getVarByName(get_id(r, v1, nbrs[i], t - 1)).get(GRB_DoubleAttr_X)<<std::endl;
            //     }
                
            // }
            // printf("%d %d\n",r,t);
            // assert(final_paths[r][t]!=nullptr);
        }
    }
}


inline Ilp3D::id_type Ilp3D::get_id(size_t r, Location3d* v1, Location3d* v2, size_t t)
{
    if(v2!=nullptr)
        return std::to_string(v2->id)+ "-"+std::to_string(v1->id)+"-" + std::to_string(r) + "-" + std::to_string(t);
    else
        return "null-"+std::to_string(v1->id)+"-" + std::to_string(r) + "-" + std::to_string(t);
}


inline void Ilp3D::store_var_for_vertices(GRBVar &var, Location3d * v1, Location3d* v2, size_t t1, size_t t2)
{
    id_type id = get_id(99999, v1, nullptr, t1);
    auto it = time_out_vector_map.find(id);
    if (it == time_out_vector_map.end())
        time_out_vector_map.insert(make_pair(id, std::vector<GRBVar>()));
    time_out_vector_map.find(id)->second.push_back(var);
    id = get_id(99999, v2, nullptr, t2);
    it = time_in_vector_map.find(id);
    if (it == time_in_vector_map.end())
        time_in_vector_map.insert(make_pair(id, std::vector<GRBVar>()));
    time_in_vector_map.find(id)->second.push_back(var);
}

inline void Ilp3D::store_var_for_robots(GRBVar &var, size_t r, Location3d* v1, Location3d* v2, size_t t1, size_t t2)
{
    id_type id = get_id(r, v1, nullptr, t1);
    auto it = robot_out_vector_map.find(id);
    if (it == robot_out_vector_map.end())
        robot_out_vector_map.insert(make_pair(id, std::vector<GRBVar>()));
    robot_out_vector_map.find(id)->second.push_back(var);
    id = get_id(r, v2, nullptr, t2);
    it = robot_in_vector_map.find(id);
    if (it == robot_in_vector_map.end())
        robot_in_vector_map.insert(make_pair(id, std::vector<GRBVar>()));
    robot_in_vector_map.find(id)->second.push_back(var);
}

inline void Ilp3D::store_var_for_edges(GRBVar &var, Location3d* v1, Location3d* v2, size_t t)
{
    id_type id = get_id(99999, v1, v2, t);
    if (v2 < v1)
        id = get_id(99999, v2, v1, t);
    auto it = edge_time_vector_map.find(id);
    if (it == edge_time_vector_map.end())
        edge_time_vector_map.insert(make_pair(id, std::vector<GRBVar>()));
    edge_time_vector_map.find(id)->second.push_back(var);
}

void get_middle_configs(Configs &starts,Configs &goals, Grids3d *graph,Configs &middle_config){
    int num_robots=starts.size();
     //cout << "Called" << endl;
    Paths3d individual_paths;
    std::vector<std::pair<size_t, size_t>> path_lengths = std::vector<std::pair<size_t, size_t>>();
    size_t max_length = 0;
    for (size_t i = 0; i < num_robots; i++)
    {
        AStarSolver solver(starts[i],goals[i]);
        solver.standard_init();
        solver.getNeighbors=[&](AStarSolver::AStarNode_p n){
            AStarSolver::AStarNodes successors;
            auto cs=graph->getNeighbors(n->v);
            cs.push_back(n->v);
            for(auto &vn:cs){
                int tn=n->t+1;                                      // tn
                int fn=n->v->manhattan_dist(goals[i])+tn;                           //fn=gn+hn     
                AStarSolver::AStarNode_p new_node=std::make_shared<AStarSolver::AStarNode>(vn,fn,tn,0,n);
                successors.push_back(new_node);
            }
            return successors;
        };
        auto pi=solver.search();

        path_lengths.push_back(std::make_pair(i, pi.size()));
        individual_paths.push_back(pi);
    }
    std::sort(path_lengths.begin(), path_lengths.end(), [](auto &left, auto &right) { return left.second > right.second; });
    middle_config.resize(num_robots,nullptr);
    for(auto &var: path_lengths)
    {
        size_t i = var.first;
        // std::cout<<"the "<<i<<" th path"<<var.second<<std::endl;
        auto desired_pt = individual_paths[i][individual_paths[i].size() / 2];
        // std::cout<<"desired point is "<<desired_pt->print()<<std::endl;
        // std::cout<<"The path is ";
        // for(auto &v :individual_paths[i]){
        //     std::cout<<v->print()<<" ";

        // }
        std::cout<<std::endl;
        std::vector<Location3d*> pool;
        auto it = find(middle_config.begin(), middle_config.end(), desired_pt);
        while (it != middle_config.end())
        {
            // if(i==2){
            //     auto nbr=graph->getNeighbors(desired_pt);
            //     // std::cout<<"debug nbr=";
            //     for(auto &v :nbr){
            //         std::cout<<v->print()<<" ";
            //     }
            //     std::cout<<std::endl;
            //     std::cout<<pool.size()<<std::endl;
            // }
            auto nbr=graph->getNeighbors(desired_pt);
            
            pool.insert(pool.end(), nbr.begin(), nbr.end());
            // std::cout<<"insert pool succeed!"<<std::endl;
            desired_pt = pool[0];
            pool.erase(pool.begin());
            it = find(middle_config.begin(), middle_config.end(), desired_pt);
        }
     
        middle_config[i] = desired_pt;
    }
}


void ilp_solve_split(Configs &starts,Configs &goals,Grids3d *graph,int split,int &makespan)
{
    int num_robots=starts.size();
    int mpieces=pow(2,split);
    std::vector<Configs> msg(mpieces+1,Configs(num_robots));
    for(int i=0;i<num_robots;i++){
        msg[0][i]=starts[i];
        msg[mpieces][i]=goals[i];
    }
    for(int i = 0; i < split; i ++){
        int numSplits = pow(2, i);
        int stepSize = pow(2, split - i);
        for(int s = 0; s < numSplits; s++){
            //System.out.println("OK");
            Configs middle;
            get_middle_configs(msg[s*stepSize],msg[(s+1)*stepSize],graph,middle);
            // int[] middle = splitPaths(g, msg[s*stepSize], msg[(s+1)*stepSize], false);
            
            for(int a = 0; a < starts.size(); a ++){
                msg[s*stepSize + stepSize/2][a] = middle[a];
            }
        }
    }

    auto solve_ilp=[](Configs &ss,Configs &gg,Grids3d *graph,int *makespan_){
        Ilp3D solver(ss,gg,graph);
        solver.solve_original();
        *makespan_=solver.makespan;
    };

    int *mk=new int[mpieces];
    for(int i=0;i<mpieces;i++){
        std::thread t(solve_ilp,std::ref(msg[i]),std::ref(msg[i+1]),graph,mk+i);
        t.join();
    }
    makespan=0;
    for(int i=0;i<mpieces;i++){
        makespan+=mk[i]-1;
    }
    makespan++;
    delete[] mk;
  

    // if (split <= 4)
    // {
    //     std::thread t1(&Ilp3D::solve_split, this,s,middle_config,split-1,std::ref(p1));
    //     std::thread t2(&Ilp3D::solve_split, this,middle_config,g,split-1,std::ref(p2));
    //     t1.join();
    //     t2.join();
    // }
    // else
    // {
    //     solver_1.solve_split(s,middle_config,split-1,p1);
    //     solver_2.solve_split(middle_config,g,split-1,p2);
    // }
    // //cout << "Combine paths" << endl;
    // result = p1;
    // result.pop_back();
    // result.insert(result.end(), p2.begin(), p2.end());
}



void save_makespan_and_time(std::string file_name,int makespan,int makespanLB,double runtime){
    std::ofstream out(file_name);
    // out<<"soc="<<soc<<std::endl;
    // out<<"lb_soc="<<socLB<<std::endl;
    out<<"makespan="<<makespan<<std::endl;
    out<<"lb_makespan="<<makespanLB<<std::endl;
    out<<"comp_time="<<(int)(runtime*1000)<<std::endl;
}