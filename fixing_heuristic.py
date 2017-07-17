# -*- coding: utf-8 -*-
"""
Created on Wed May 17 17:10:04 2017

@author: Tamke
"""

import numpy as np
from gurobipy import *
from createModel import buildModel_VRPD
# Callback - use lazy constraints to eliminate sub-tours

def subtourelim(model_tsp, where):
    if where == GRB.Callback.MIPSOL:
        # make a list of edges selected in the solution
        vals = model_tsp.cbGetSolution(model_tsp._vars)
        selected = tuplelist((i,j) for i,j in model_tsp._vars.keys() if vals[i,j] > 0.5)
        # find the shortest cycle in the selected edge list
        tour = subtour(selected, model_tsp._tsp_nodes, model_tsp._n)
        if len(tour) < model_tsp._n:
            # add subtour elimination constraint for every pair of cities in tour
            model_tsp.cbLazy(quicksum(model_tsp._vars[i,j] for i,j in itertools.permutations(tour, 2)) <= len(tour)-1)

# Given a tuplelist of edges, find the shortest subtour

def subtour(edges, fixed_tsp_nodes, n):
    unvisited = list(fixed_tsp_nodes)
    cycle = range(n+1) # initial length has 1 more city
    while unvisited: # true if list is non-empty
        thiscycle = []
        neighbors = unvisited
        while neighbors:
            current = neighbors[0]
            thiscycle.append(current)
            unvisited.remove(current)
            neighbors = [j for i,j in edges.select(current,'*') if j in unvisited]
        if len(cycle) > len(thiscycle):
            cycle = thiscycle
    return cycle
def two_opt_fixing_heuristic(vehicles, validInequalities, nodes, uav_serviceable, P, velocities, customers, operatingTimes, M, fixed_nodes):
    truck_nodes_fixed = fixed_nodes[0]
    drone_nodes_fixed = fixed_nodes[1]
    tau = velocities[0]
    m = buildModel_VRPD(vehicles, validInequalities, nodes, uav_serviceable, P, velocities, customers, operatingTimes, M, False, False)
    MIPStart = {}
#        ======================================================================================================================================================================== FT
    m.update()
#    m.write("LP.LP")
    for f in vehicles:
        if len(drone_nodes_fixed[f]) > np.ceil(customers/2):
            truck_nodes_fixed[f].append(drone_nodes_fixed[f][0])
            drone_nodes_fixed[f].pop(0)
##### TSP FOR TRUCK NODES #####
        fixed_tsp_nodes = list(truck_nodes_fixed[f])
        fixed_tsp_nodes.insert(0,0)
        n = len(fixed_tsp_nodes)
        dist_tsp = {}
        for i in fixed_tsp_nodes:
            for j in fixed_tsp_nodes[1:]:
                if i != j:
                    dist_tsp[i,j] = tau[i,j]
        for i in fixed_tsp_nodes[1:]:
            dist_tsp[i,0] = tau[i,customers+1]
        m_tsp = Model('model_tsp')
#        m_tsp.setParam('LogFile', "logfil.log")
        m_tsp.Params.OutputFlag = 0
        m_tsp.Params.Threads = 2
###       Create variables
        vars = m_tsp.addVars(dist_tsp.keys(), obj=dist_tsp, vtype=GRB.BINARY, name='e')
        m_tsp.addConstrs(vars.sum(i,'*') == 1 for i in fixed_tsp_nodes)
        m_tsp.addConstrs(vars.sum('*',i) == 1 for i in fixed_tsp_nodes)
        m_tsp._vars = vars
        m_tsp.Params.lazyConstraints = 1
        m_tsp._tsp_nodes = fixed_tsp_nodes
        m_tsp._n = n
        m_tsp.update()
#        m_tsp.write('MODEL_TSP.LP')
        m_tsp.optimize(subtourelim)
        vals = m_tsp.getAttr('x', vars)
    #   m_tsp.write('Model_TSP.Sol')
        selected = tuplelist((i,j) for i,j in vals.keys() if vals[i,j] > 0.5)
        tour = subtour(selected, fixed_tsp_nodes, n)
        assert len(tour) == n
    for f in vehicles:
        for i in truck_nodes_fixed[f]:
            m.getVarByName('q[' + str(i) + ',' + str(f) +']').setAttr('LB', 1)
            for j in truck_nodes_fixed[f]:
                if i != j:
                    if vals[i,j] > 0.5:
                        m.getVarByName('x[' + str(i) + ',' + str(j) + ',' + str(f) +']').setAttr('LB', 1)
            m.getVarByName('w[' + str(i) + ',' + str(f) +']').setAttr('LB', 0)
        for i in drone_nodes_fixed[f]:
            m.getVarByName('w[' + str(i) + ',' + str(f) +']').setAttr('LB', 1)
            m.getVarByName('q[' + str(i) + ',' + str(f) +']').setAttr('LB', 0)
    m.setParam('Threads', 4)
    m.setParam('CutPasses',1)
    m.setParam('Aggregate', 0)
    m.optimize()
    if m.Status == GRB.OPTIMAL:
        for f in vehicles:
            best_objective = 1000000
            truck_nodes_fixed[f].clear()
            drone_nodes_fixed[f].clear()
            for i in nodes[1:-1]:
                var = m.getVarByName('q[' + str(i) + ',' + str(f) +']')
                if var.getAttr('x') > 0.5:
                    truck_nodes_fixed[f].append(i)
                if i in uav_serviceable:
                    var = m.getVarByName('w[' + str(i) + ',' + str(f) +']')
                    if var.getAttr('x') > 0.5:
                        drone_nodes_fixed[f].append(i)
            for var in m.getVars():
                MIPStart[var.getAttr('VarName')] = var.getAttr('X')
            improvement_found = True
            move_j = -1
            move_k = -1
            best_objective = m.ObjVal
            runtime_models = 0
            n_iteration_models = 0
            while improvement_found:
                improvement_found = False
                for j in range(len(truck_nodes_fixed[f])):
                    if improvement_found:
                        break
                    for k in range(len(drone_nodes_fixed[f])):
                        if (j != move_k or k != move_j):
                            tmp_m = m.copy()
                            tmp_truck_nodes_fixed = truck_nodes_fixed[f].copy()
                            tmp_drone_nodes_fixed = drone_nodes_fixed[f].copy()
                            tmp_truck_nodes_fixed[j], tmp_drone_nodes_fixed[k] = tmp_drone_nodes_fixed[k], tmp_truck_nodes_fixed[j]
                            for v in tmp_m.getVars():
                                v.LB = 0
                            ##### TSP FOR TRUCK NODES #####
                            fixed_tsp_nodes = list(tmp_truck_nodes_fixed)
                            fixed_tsp_nodes.insert(0,0)
                            n = len(fixed_tsp_nodes)
                            dist_tsp = {}
                            for iter_i in fixed_tsp_nodes:
                                for iter_j in fixed_tsp_nodes[1:]:
                                    if iter_i != iter_j:
                                        dist_tsp[iter_i,iter_j] = tau[iter_i,iter_j]
                            for iter_i in fixed_tsp_nodes[1:]:
                                dist_tsp[iter_i,0] = tau[iter_i,customers+1]
                            m_tsp = Model('model_tsp')
                    #        m_tsp.setParam('LogFile', "logfil.log")
                            m_tsp.Params.OutputFlag = 0
                            m_tsp.Params.Threads = 4
                    ###       Create variables
                            vars = m_tsp.addVars(dist_tsp.keys(), obj=dist_tsp, vtype=GRB.BINARY, name='e')
                            m_tsp.addConstrs(vars.sum(i,'*') == 1 for i in fixed_tsp_nodes)
                            m_tsp.addConstrs(vars.sum('*',i) == 1 for i in fixed_tsp_nodes)
                            m_tsp._vars = vars
                            m_tsp.Params.lazyConstraints = 1
                            m_tsp._tsp_nodes = fixed_tsp_nodes
                            m_tsp._n = n
                            m_tsp.optimize(subtourelim)
                            vals = m_tsp.getAttr('x', vars)
                            if m_tsp.ObjVal < best_objective:                            
                                for i in tmp_truck_nodes_fixed:
                                    tmp_m.getVarByName('q[' + str(i) + ',1]').setAttr('LB', 1)
                                    tmp_m.getVarByName('w[' + str(i) + ',1]').setAttr('LB', 0)
                                    for iter_j in tmp_truck_nodes_fixed:
                                        if i!=iter_j:
                                            if vals[i,iter_j] > 0.5:
                                                tmp_m.getVarByName('x[' + str(i) + ',' + str(iter_j) + ',' + str(f) +']').setAttr('LB', 1)
                                for i in tmp_drone_nodes_fixed:
                                    tmp_m.getVarByName('w[' + str(i) + ',1]').setAttr('LB', 1)
                                    tmp_m.getVarByName('q[' + str(i) + ',1]').setAttr('LB', 0)
                                tmp_m.Params.OutputFlag = 0
                                tmp_m.setParam('CutPasses', 1)
                                tmp_m.setParam('Cutoff', best_objective)
                                tmp_m.optimize()
                                runtime_models += tmp_m.runtime
                                n_iteration_models += 1
                                if tmp_m.Status == GRB.OPTIMAL:
                                    if tmp_m.ObjVal < best_objective - 0.01:
                                        truck_nodes_fixed[f] = tmp_truck_nodes_fixed
                                        drone_nodes_fixed[f] = tmp_drone_nodes_fixed
                                        move_j = j
                                        move_k = k
                                        for var in tmp_m.getVars():
                                            MIPStart[var.getAttr('VarName')] = var.getAttr('X')
                                        m = tmp_m.copy()
                                        best_objective = tmp_m.ObjVal
                                        print(best_objective)
                                        improvement_found = True
                                        break
    elif m.Status == GRB.INFEASIBLE:
        m.reset()
        for i in drone_nodes_fixed:
            m.getVarByName('w[' + str(i) + ',1]').setAttr('LB', 0)
            m.optimize()
    M = best_objective
    return_value = [M, MIPStart]
    print(runtime_models)
    print(n_iteration_models)
    return return_value