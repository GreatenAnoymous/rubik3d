/**
 * @file labp.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */

// Linear bottleneck assignment
#ifndef LBAP_H
#define LBAP_H

#include<algorithm>
#include<iostream>
#include<vector>
#include<unordered_map>


#define BIG 10000


double labp_solve(std::vector <std::vector<double> >& costMatrix, std::vector<int>& Assignment);

double lba_sparse(std::vector<std::tuple<int,int,double>>&costEdges, std::vector<int>& Assignment); //for solving sparse matrix

double lap(int dim,std::vector<std::vector<double>> &costs,std::vector<int>&rowsol,std::vector<int> &colsol,std::vector<double> &u,std::vector<double> &v);

double lap_sparse(std::vector<std::tuple<int,int,double>>&costEdges, std::vector<int>& Assignment);
#endif