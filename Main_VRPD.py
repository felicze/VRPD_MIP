# -*- coding: utf-8 -*-
"""
Created on Thu Apr 27 16:03:36 2017

@author: tamke
"""


# coding: utf-8

"""
Exact testing
"""
#import os
#import sys

#from bokeh.plotting import figure, output_file, show
#from bokeh.models import Arrow, OpenHead, NormalHead, VeeHead, ColumnDataSource, Range1d, LabelSet, Label
#import string
#import operator
from gurobipy import *
from createModel import buildModel_VRPD, buildModel_TSPD, changeObjective_to_minDistance
from fixing_heuristic import two_opt_fixing_heuristic
import time
import os
from sklearn.cluster import KMeans
import math
import itertools
import numpy as np
import pandas as pd
import networkx as nx
import random
import matplotlib.pyplot as plt
results = pd.DataFrame()

def Variablen_auslesen(variablen):
    """Liest die Werte der Entscheidungsvariablen aus. Es könnnen eine aber
    auch mehrere Variablen übergeben werden."""
    if type(variablen) == list:
        for dicts in variablen:
            for k in dicts:
                try:
                    dicts[k] = round(dicts[k].X)
                except AttributeError:
                    dicts[k] = dicts[k]
    else:
        dicts = variablen
        for k in dicts:
            try:
                dicts[k] = round(dicts[k].X)
            except AttributeError:
                dicts[k] = dicts[k]

def file_len(fname):
    """Zählt die Zeilen einer .txt Datei."""
    with open(fname) as f:
        for i, l in enumerate(f):
            pass
    return i + 1

# Callback - use lazy constraints to eliminate sub-tours

def subtourelim(model_tsp, where):
    if where == GRB.Callback.MIPSOL:
        # make a list of edges selected in the solution
        vals = model_tsp.cbGetSolution(model_tsp._vars)
        selected = tuplelist((i,j) for i,j in model_tsp._vars.keys() if vals[i,j] > 0.5)
        # find the shortest cycle in the selected edge list
        tour = subtour(selected)
        if len(tour) < n:
            # add subtour elimination constraint for every pair of cities in tour
            model_tsp.cbLazy(quicksum(model_tsp._vars[i,j] for i,j in itertools.permutations(tour, 2)) <= len(tour)-1)

# Given a tuplelist of edges, find the shortest subtour

def subtour(edges):
    unvisited = list(tsp_nodes)
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

def getTotalDistance(model, vehicles, nodes, velocities):
    totalDistance = 0
    tau = velocities[0]
    for i in nodes[:-1]:
        for j in nodes[1:]:
            if i != j:
                for f in vehicles:
                    var = model.getVarByName('x[' + str(i) + ',' + str(j) + ',' + str(f) + ']')
                    if var.X > 0.5:
                        totalDistance += tau[i,j]
    return totalDistance

sl = 0
sr = 0
e = 1000 * 10**0

n_ValidInequalities = 7
n_Combinations = 2**n_ValidInequalities
start = 0
end = n_Combinations
start_combinations = 0
#vi_combinations = [[0 for j in range(n_ValidInequalities)] for i in range(n_Combinations)]
#vi_combinations = [[0 for j in range(n_ValidInequalities)] for i in range(11)]
#for i in range(start,end):
#    tmp = '{0:07b}'.format(i)
##    print(tmp)
#    vi_combinations[i-start] = [int(j) for j in tmp]
#    print(vi_combinations[i])
#for file in os.listdir('C:\\Users\\manzke\\Desktop\\Instances_larger'):
vi_combinations = [
    [1,1,0,0]
#    [1,0,0,0]
#    [1,1,0,0]
#    [1,0,1,0],
#    [1,1,1,0]
    ]
#vi_combinations = [
#    [1,0,0,1,1,0,0],
#    [1,0,0,1,0,0,0]
#    ]
#print(vi_combinations)
#path = 'all_Instances_VRP_TSP'
path = 'Test'
customers = 15
save_file = 'TestConfig_' + str(customers) +'_Customers_V25_Test2.csv'
#save_file = 'M_small_VRPD_Test2.csv'
results = pd.concat([results,pd.DataFrame(['Config', 'Instance', 'Iteration', 'Vehicles','Customers',
                                           'v_truck', 'v_drone', 'CompletionTime', 'TimeHeuristic', 'TimeModel_CompletionTime', 'TimeModel_TotalDistance', 'TotalTime', 'M', 'TotalDistance_CT', 'TotalDistance_min', 'VehiclesUsed', 'DronesUsed', 'Gap_CT', 'Gap_TD']).T])
#for instance in os.listdir(path):
#    if 'n9' in instance or 'n10' in instance:
for F in [1]:
    for validInequalities in vi_combinations:
        vi = 'VI_'
        for i in validInequalities:
            if i == 1:
                vi += '+'
            else:
                vi += '-'
        start_heuristic = True
        #        for instance in [2,4,7,13,14]:
        #        for instance in [7]:
        for instance in [1]:
#        for instance in range(7):
                    try:
                        n_iterations = 1
                        k = F
        #                generate instances
                        instance_bouman = False
                        if instance_bouman:
                            file = instance
                            name = "%s\\%s" % (path,file)
                            info = np.genfromtxt(name, delimiter = '\n',comments = '/*', skip_footer = file_len(name)-8)
                            txt = np.genfromtxt(name, delimiter = ' ',comments = '/*', skip_header = 6, usecols = (0,1))
                            c_t = info[0]
                            c_d = info[1]
                            customers = int(info[2]-1)
                            nodes = range(0,customers+2)
                #                   depot + customers
                            x_loc = [a for [a,b] in txt]
                            y_loc = [b for [a,b] in txt]
                #                   depot
                            x_loc.append(x_loc[0])
                            y_loc.append(y_loc[0])
                            locations = [(x_loc[j], y_loc[j]) for j in range(len(x_loc))]
                        else:
                            c_t = 1
                            c_d = 0.50
                            nodes = range(0,customers+2)
                            random.seed((instance+1+customers))
                            depot = (random.uniform(0,1), random.uniform(0,1))
        #                    depot = (0,0)
        #                    locations = [(10, 80), (20, 60), (50, 40), (40,30), (30,90), (40,70), (80,30), (90,20)]
                            locations = [(random.randint(0,100), random.randint(0,100)) for i in range(customers)]
                            locations.insert(0, depot)
                            locations.insert(customers+2, depot)

                        departureNodes = list(nodes[:-1])
                        arrivalNodes= list(nodes[1:])
                        vehicles = range(1, k + 1)
                        uav_serviceable = nodes[1:-1]


                        tau = {}
                        tau_d = {}
                        dist = {(i,j) :
                            1*round(np.sqrt(sum((locations[i][k] - locations[j][k])**2 for k in range(2))),2)
                            for i in departureNodes for j in arrivalNodes}
                        dist[0,customers+1] = 0.0
                        dist[0,0] = 0.0
                        dist[customers+1,customers+1] = 0.0
                        arcs, velo1, velo2 = multidict({
                         (i,j) :
                            [1*round(math.sqrt(sum((locations[i][k] - locations[j][k])**2 for k in range(2))) * c_t, 2),
                             1*round(math.sqrt(sum((locations[i][k] - locations[j][k])**2 for k in range(2))) * c_d, 2)]
                            for i in departureNodes for j in arrivalNodes
                         })
                        arcs.append((0,0))
                        arcs.append((0,customers+1))
                        arcs.append((customers+1, customers+1))
                        velo1[(0,0)] = 0.0
                        velo1[(0,customers+1)] = 0.0
                        velo1[(customers+1,customers+1)] = 0.0
                        velo2[(0,0)] = 0.0
                        velo2[(0,customers+1)] = 0.0
                        velo2[(customers+1,customers+1)] = 0.0

                        for key in dist:
                            tau[key] = dist[key] * c_t
                            tau_d[key] = dist[key] * c_d

                        P = []
                        for i in departureNodes:
                            for j in uav_serviceable:
                                if j != i:
                                    for k in arrivalNodes:
                                        if k != j and k != i and tau_d[i,j] + tau_d[j,k] <= e:
                                            P.append([i,j,k])

        #            ==============================================================================
        #            ==============================================================================
        #            ==============================================================================
        #             MODEL FOR TSP (without drone)
        #            ==============================================================================
        #            ==============================================================================
        #            ==============================================================================
                        print('######### START TSP WITH SINGLE DRONE FLIGHT #########')
                        start_phase_one = time.clock()
                        MIPStart = {}
                        kmeans = KMeans(n_clusters=F)
                        kmeans.fit(np.array(locations)[1:-1])
                        labels = kmeans.labels_
                        truck = {}
                        drone = {}
                        M_truck = {}
                        for ix in np.unique(labels):
                            M_truck[ix+1] = math.inf
                            cluster_customers = [j+1 for j in range(len(labels)) if labels[j] == ix]
                            if len(cluster_customers) > 1:
                                cluster_drone_customers = (j for j in cluster_customers if j in uav_serviceable and [0,j,customers+1] in P)
                                for drone_node in cluster_drone_customers:
                                    tsp_nodes = list(cluster_customers)
                                    tsp_nodes.insert(0,0)
                                    tsp_nodes.remove(drone_node)
                                    n = len(cluster_customers)
                                    dist_tsp = {}
                                    for i in tsp_nodes:
                                        for j in tsp_nodes[1:]:
                                            if i != j:
                                                dist_tsp[i,j] = tau[i,j]
                                    for i in tsp_nodes[1:]:
                                        dist_tsp[i,0] = tau[i,customers+1]
                                    m_tsp = Model('model_tsp')
        #                            m_tsp.setParam('LogFile', "logfil.log")
                                    m_tsp.Params.OutputFlag = 0
                                    m_tsp.Params.Threads = 4
        #                            Create variables
                                    vars = m_tsp.addVars(dist_tsp.keys(), obj=dist_tsp, vtype=GRB.BINARY, name='e')
                                    m_tsp.addConstrs(vars.sum(i,'*') == 1 for i in tsp_nodes)
                                    m_tsp.addConstrs(vars.sum('*',i) == 1 for i in tsp_nodes)
                                    m_tsp._vars = vars
                                    m_tsp.Params.lazyConstraints = 1
                                    m_tsp.update()
        #                            m_tsp.write('MODEL_TSP.LP')
                                    m_tsp.optimize(subtourelim)
                                    vals = m_tsp.getAttr('x', vars)
#                                    m_tsp.write('Model_TSP.Sol')
                                    selected = tuplelist((i,j) for i,j in vals.keys() if vals[i,j] > 0.5)
                                    tour = subtour(selected)
                                    assert len(tour) == n
                                    min_time = max(m_tsp.objVal, tau_d[0,drone_node] + tau_d[drone_node,customers+1])
                                    if min_time < M_truck.get(ix+1):
                                        M_truck[ix+1] = min_time
                                        drone[ix+1] = (0,drone_node,customers+1,ix+1)
                                        truck[ix+1] = []
                                        for i,j in vars.keys():
                                            if vars[i,j].getAttr('X') > 0.5:
                                                truck[ix+1].append((i,j,ix+1))
                            else:
                                node = cluster_customers[0]
                                M_truck[ix+1] = tau_d[0,node] + tau_d[node,customers+1]
                                truck[ix+1] = []
                                drone[ix+1] = (0,node,customers+1,ix+1)
                        for f in vehicles:
                            for i,j,k in truck[f]:
                                if j == 0:
                                    j = customers+1
                                MIPStart['x[' + str(i) + ',' + str(j) + ',' + str(k) + ']'] = 1
                            MIPStart['y[' + str(0) + ',' + str(drone[f][1]) + ',' + str(customers+1) + ',' + str(f) + ']'] = 1
                        M = max(M_truck.values())
                        velocities = [velo1, velo2]
                        operatingTimes = [sr, sl, e]
        #            ==============================================================================
        #            ==============================================================================
        #            ==============================================================================
        #             VPRD
        #            ==============================================================================
        #            ==============================================================================
        #            ==============================================================================
                        if start_heuristic:
                            print('######### START HEURISTIC #########')
                            timeModelOne = 0
                            nodes_in_cluster = {1: list(nodes)}
                            number_clusters = 1
                            MIPStart_tmp = {}
                            MIPStart.clear()
                            truck_nodes_fixed = {}
                            drone_nodes_fixed = {}
                            if F == 1:
                                while max(len(v) for v in nodes_in_cluster.values()) > 9:
                                    number_clusters += 1
                                    kmeans = KMeans(n_clusters=number_clusters)
                                    kmeans.fit(np.array(locations)[1:-1])
                                    labels = kmeans.labels_
                                    for ix in np.unique(labels):
                                        nodes_in_cluster[ix+1] = [j+1 for j in range(len(labels)) if labels[j] == ix]
                            else:
                                kmeans = KMeans(n_clusters=F)
                                kmeans.fit(np.array(locations)[1:-1])
                                labels = kmeans.labels_
                                for ix in np.unique(labels):
                                    nodes_in_cluster[ix+1] = [j+1 for j in range(len(labels)) if labels[j] == ix]
                            for k,v in nodes_in_cluster.items():
                                n_customers_in_cluster = len(v)
                                v.insert(0, nodes[0])
                                v.append(nodes[customers+1])
                                new_velocities = velocities.copy()
                                cluster_drone_customers = [j for j in v if j in uav_serviceable and [0,j,customers+1] in P]
                                tmp_Model = buildModel_VRPD([k], validInequalities, v, cluster_drone_customers, P, new_velocities, len(v)-2, operatingTimes, M, False, False)
                                tmp_Model.setParam('Threads', 2)
                                tmp_Model.setParam('CutPasses',1)
                                tmp_Model.optimize()
#                                tmp_Model.write('Model' + str(k) + '.LP')
#                                tmp_Model.write('Solution' + str(k) + '.SOL')
                                if tmp_Model.Status == GRB.Status.OPTIMAL:
                                    MIPStart_tmp[k] = {}
                                    for var in tmp_Model.getVars():
                                        if var.getAttr('vtype') == GRB.BINARY:
                                            MIPStart_tmp[k][var.getAttr('VarName')] = var.getAttr('X')
                                    M_truck[k] = tmp_Model.ObjVal
                                    truck_nodes_fixed[k] = []
                                    drone_nodes_fixed[k] = []
                                    for i in v[1:-1]:
                                        var = tmp_Model.getVarByName('q[' + str(i) + ',' + str(k) + ']')
                                        if var.getAttr('X') > 0.5:
                                            truck_nodes_fixed[k].append(i)
                                        if i in uav_serviceable:
                                            var = tmp_Model.getVarByName('w[' + str(i) + ',' + str(k) + ']')
                                            if var.getAttr('x') > 0.5:
                                                drone_nodes_fixed[k].append(i)
                            fixed_nodes = [truck_nodes_fixed, drone_nodes_fixed]
                            f = 1
                            for l in sorted(truck_nodes_fixed, key=lambda k: len(truck_nodes_fixed[k]), reverse=True):
                                for name in MIPStart_tmp[l]:
                                    splitted_name = name.split(",")
                                    new_var_name = ','.join(splitted_name[:-1]) + "," + str(f) + "]"
                                    MIPStart[new_var_name] = MIPStart_tmp[l][name]
                                f += 1
                            M = max(M_truck.values())
                            two_opt_time = 0
                        ###### TWO OPT  ######
                            if F == 1:
                                print('######### START TWO OPT #########')
                                truck_nodes_merged = {}
                                truck_nodes_merged[1] = []
                                drone_nodes_merged = {}
                                drone_nodes_merged[1] = []
                                for f in truck_nodes_fixed:
                                    truck_nodes_merged[1] += truck_nodes_fixed[f]
                                    drone_nodes_merged[1] += drone_nodes_fixed[f]
                                fixed_nodes = [truck_nodes_merged, drone_nodes_merged]
                                MIPStart.clear()
                                for f in truck_nodes_fixed:
                                    for i in truck_nodes_fixed[f]:
                                        MIPStart['q[' + str(i) + ',' + str(f) + ']'] = 1
                                    for i in drone_nodes_fixed[f]:
                                        MIPStart['w[' + str(i) + ',' + str(f) + ']'] = 1
                                result_fixing = two_opt_fixing_heuristic(vehicles, validInequalities, nodes, uav_serviceable, P, velocities, customers, operatingTimes, 800, fixed_nodes) # M ändern
                                M = result_fixing[0]
                                MIPStart = result_fixing[1]
                        ###### TWO OPT  ######
                        end_phase_one = time.clock()
                        phase_one_time = end_phase_one - start_phase_one
                        print('Zeit Erzeugung Startlösung: ', phase_one_time)
        #               ###### END HEURISTIC ######
        #                m_final = buildModel_VRPD(vehicles, validInequalities, nodes, uav_serviceable, P, velocities, customers, operatingTimes, M, False)
        #                m_final.Params.OutputFlag = 1
        #                m_final.setParam('Threads', 8)
        #                m_final.setParam('CutPasses', 0)
        #                m_final.setParam('Heuristics', 0.01)
        #                m_final.setParam('Cuts', 2)
        #                m_final.update()
        #                m_final.setParam(GRB.param.TuneTimeLimit, 24*3600)
        #                m_final.setParam(GRB.param.TuneTrials, 6)
        #                m_final.setParam(GRB.param.TuneCriterion, 1)
        #                m_final.setParam(GRB.param.TuneResults, 5)
        #                m_final.tune()
        #                m_final.write('Model_finalTest.LP')
                        for iteration in range(n_iterations):
                            print('######################################')
                            print('######### SOLVE MODEL OBJ CT #########')
                            print('######################################')
                            lb_ObjVal = 0
                            for j in nodes[1:-1]:
                                if 2*tau_d[0,j] > lb_ObjVal:
                                    lb_ObjVal = 2*tau_d[0,j]
                            lb_ObjVal += 0.001
                            m_final = buildModel_VRPD(vehicles, validInequalities, nodes, uav_serviceable, P, velocities, customers, operatingTimes, M, False, False)
                            m_final.Params.OutputFlag = 1
                            m_final.setParam('Threads', 4)
                            m_final.setParam('CutPasses', 0)
                            m_final.setParam('Aggregate', 0)
        #                    m_final.setParam('Symmetry', 2)
        #                    m_final.setParam('MIPFocus', 3)
                            m_final.setParam('TimeLimit', 7200)
        #                    m_final.setParam('BestObjStop', lb_ObjVal)
                            if start_heuristic:
                                for key, value in MIPStart.items():
                                    var = m_final.getVarByName(key)
                                    if var.getAttr('vtype') == GRB.BINARY:
                                        var.setAttr('Start', value)
                            seed = np.ceil((time.time() % 100000) * random.uniform(0,10))
                            m_final.setParam('seed', seed)
                            m_final.update()
        #                    m_final.update()
        #                    m_final_relaxed = m_final.relax()
        #                    m_final_relaxed.optimize()
        #                    m_final_relaxed.write('SolRelaxed.SOL')
        #                    m_final.write('Model_finalTest.LP')
                            m_final.optimize()
                            m_final.write('Model_CT_1.SOL')
                            completionTime = m_final.ObjVal
                            if m_final.Status == GRB.OPTIMAL or m_final.Status == GRB.TIME_LIMIT:
                                gap_CompletionTime = m_final.MIPGap
                            elif m_final.Status == GRB.USER_OBJ_LIMIT or m_final.ObjVal < lb_ObjVal:
                                gap_CompletionTime = -1
                            cT_totalDistance = getTotalDistance(m_final, vehicles, nodes, velocities)
                            time_CompletionTime = m_final.RunTime
                            if F > 1:
                                print('######################################')
                                print('######### SOLVE MODEL OBJ TD #########')
                                print('######################################')
                                m_final.setParam('BestObjStop', -GRB.INFINITY)
                                variables = m_final.getVars()
                                for var in variables:
                                    var.Start = var.getAttr('X')
                                m_final = changeObjective_to_minDistance(m_final, vehicles, nodes, velocities)
                                m_final.setParam('TimeLimit', 7200)
                                m_final.update()
        #                        m_final_relaxed = m_final.relax()
        #                        m_final_relaxed.optimize()
        #                        m_final_relaxed.write('SolRelaxed.SOL')
                                m_final.optimize()
                                m_final.write('Model_TD_2.SOL')
                                time_totalDistance = m_final.RunTime
                                min_totalDistance = m_final.ObjVal
                                gap_totalDistance = m_final.MIPGap
                            else:
                                time_totalDistance = 0
                                min_totalDistance = m_final.ObjVal
                                gap_totalDistance = 0
                            totalTime = time_CompletionTime + time_totalDistance + phase_one_time
                            try:
                                n_used_vehicles = 0
                                n_used_drones = 0
                                b_vehicle_used = False
                                for f in vehicles:
                                    b_drone_used = False
                                    for i in nodes[1:-1]:
                                        var = m_final.getVarByName('x[0,' + str(i) + ',' + str(f) + ']')
                                        if var.getAttr('X') > 0.5:
                                            n_used_vehicles += 1
                                    for i in nodes[:-1]:
                                        if b_drone_used == True:
                                            break
                                        else:
                                            for j in nodes[1:-1]:
                                                if b_drone_used == True:
                                                    break
                                                elif i != j:
                                                    for k in nodes[1:]:
                                                        if i != k and j != k:
                                                            var = m_final.getVarByName('y[' + str(i) + ',' + str(j) + ',' + str(k) + ',' + str(f) + ']')
                                                            if var.getAttr('X') > 0.5:
                                                                n_used_drones += 1
                                                                b_drone_used = True
                                                                break

                                results = pd.concat([results,pd.DataFrame(['VRPD', str(instance), str(iteration), str(F),str(customers),
                                                                           c_t,c_d,
                                                                           str(round(completionTime,2)),
                                                                           str(round(phase_one_time,2)), str(round(time_CompletionTime, 2)), str(round(time_totalDistance,2)), str(round(totalTime,2)),
                                                                           str(round(M,2)), str(round(cT_totalDistance,2)), str(round(min_totalDistance,2)),
                                                                           str(n_used_vehicles),
                                                                           str(n_used_drones),
                                                                           str(round(gap_CompletionTime,4)), str(round(gap_totalDistance,4))]).T])

                                results.to_csv('Results\\%s' %save_file , sep = ';', dec = ',', index=False, header=False)
                                m_final.reset()
#                                G=nx.Graph()
#                                dict_loc ={i: locations[i] for i in range(len(locations)-1)}
#                                for n, p in dict_loc.items():
#                                    G.add_node(n, pos = p)
#                                nx.draw(G, dict_loc, with_labels=True, node_size=0)
                            except GurobiError:
                                continue

                            tour_truck = {}
                            locseq_truck = {}
                            locseq_drone = {}
        #                    d1 = {}
        #                    d2 = {}
        #                    for f in vehicles:
        #                        list_f = []
        #                        node = 0
        #                        list_f.append(node)
        #                        while node != customers + 1:
        #                            for j in arrivalNodes:
        #                                if j != node and x[node,j,f] > 0.5:
        #                                    list_f.append(j)
        #                                    node = j
        #                        tour_truck[f] = list_f
        #                        locseq_truck[f] = [locations[k] for k in tour_truck[f]]
        #                        d1[f], d2[f] = zip(*locseq_truck[f])
        #                        locseq_drone[f] = locseq_drone.get(f, {})
        #                        for k in route_d[f]:
        #                            locseq_drone[f][k] = [locations[i] for i in route_d[f][k]]
        #                    colors = ["blue", "red", "yellow"]
        #                    output_file("graph_solution.html")
        #                    dim1, dim2 = zip(*locations)
        #                    source = ColumnDataSource(data=dict(x=dim1[1:], y=dim2[1:], names=range(1,customers+2), time=[t[i,f] for i in arrivalNodes for f in vehicles]))
        #                    graph_solution = figure(title="test", x_range=[-10,110], y_range=[-10,110])
        #                    graph_solution.scatter(x='x', y='y', source=source, size=6)
        #                    labels = LabelSet(x='x', y='y', text='names', level='glyph', x_offset=10, y_offset=0, source=source, render_mode='canvas')
        #                    graph_solution.add_layout(labels)
        #                    for f in vehicles:
        #                        lab_time = LabelSet(x='x', y='y', text='time', level='glyph', x_offset=-20, y_offset=0, source=source, render_mode='canvas')
        #                        graph_solution.add_layout(lab_time)
        ##                        graph_solution.circle(d1[f],d2[f],size=6)
        #                        graph_solution.line(d1[f],d2[f], line_color=colors[f-1])
        #                        for k in locseq_drone[f]:
        #                            temp_d1, temp_d2 = zip(*locseq_drone[f][k])
        ##                            graph_solution.circle(d1[1],d2[1],size=6)
        #
        #                            graph_solution.line(temp_d1,temp_d2, line_dash='dashed', line_color=colors[f-1])
        ##                    show(graph_solution)
        #                    m_final.reset()
        #                    results.to_pickle('C:\\Users\\manzke\\Desktop\\exact_1112.pkl')
                    except:
                        IOError
                    continue

#results.columns = ['instance','number of vehicles','customers','truck cost', 'drone cost',                       'truckroute', 'droneroute', 'optimum', 'time']
#results.loc[:,('truck cost',  'drone cost', 'optimum', 'time')] =                         results.loc[:,('truck cost',  'drone cost',                                        'optimum', 'time')].astype(float)
#results.loc[:,('number of vehicles','customers')] =                         results.loc[:,('number of vehicles','customers')].astype(int)
#results = results.applymap(str).replace(r'\.',',',regex=True)
#results.to_csv('C:\\Users\\manzke\\Desktop\\exact_all_1112.csv', sep = ';', dec = ',', index=False)
