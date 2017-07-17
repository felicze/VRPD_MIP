# -*- coding: utf-8 -*-
"""
Created on Mon Mar 13 15:30:45 2017

@author: tamke
"""

import numpy as np
from gurobipy import *

def buildModel_VRPD(vehicles, validInequalities, nodes, uav_serviceable, P, velocities, customers, operatingTimes, bigM, coord_relax, min_totalDistance):
    if validInequalities == None:
        vi = False
        lb = False
    else:
        vi = True
        lb = True
    departureNodes = list(nodes[:-1])
    arrivalNodes= list(nodes[1:])
    startDepot = nodes[0]
    endDepot = nodes[-1]

    tau = velocities[0]
    tau_d = velocities[1]
    sr = operatingTimes[0]
    sl = operatingTimes[1]
    e = operatingTimes[2]
#   ==============================================================================
#   ==============================================================================
#   ==============================================================================
#   MODEL
#   ==============================================================================
#   ==============================================================================
#   ==============================================================================

    modelVRPD = Model('model')

#    ==============================================================================
#    ==============================================================================
#    ==============================================================================
#    VARIABLES
#    ==============================================================================
#    ==============================================================================
#    ==============================================================================
    q = modelVRPD.addVars(nodes, vehicles, vtype = GRB.BINARY, name = 'q')
    w = modelVRPD.addVars(nodes[1:-1], vehicles, vtype = GRB.BINARY, name = 'w')
    x = modelVRPD.addVars(departureNodes, arrivalNodes, vehicles, vtype = GRB.BINARY, name = 'x')
    y = modelVRPD.addVars(departureNodes, nodes[1:-1], arrivalNodes, vehicles, vtype = GRB.BINARY, name = 'y')
    p = modelVRPD.addVars(nodes[1:-1], nodes[1:-1], vehicles, vtype = GRB.BINARY, name = 'p')
    u = modelVRPD.addVars(nodes[1:-1], vehicles, vtype = GRB.CONTINUOUS, lb = 0.0, ub = customers, name = 'u')
    t = modelVRPD.addVars(nodes, vehicles, vtype = GRB.CONTINUOUS, name = 't')
    z = modelVRPD.addVar(vtype = GRB.CONTINUOUS, name = 'z')
#    b = modelVRPD.addVars(nodes[1:-1], nodes[1:-1], vehicles, vtype = GRB.BINARY, name = 'b')
    b = tupledict()
    for idx, i in enumerate(nodes[1:-2]):
        for j in nodes[idx+2:-1]:
            if i != j:
                for f in vehicles:
                    b[i,j,f] = modelVRPD.addVar(vtype=GRB.BINARY, name = 'b[%d,%d,%d]'%(i,j,f))
    modelVRPD.update()
    for i in nodes[1:-1]:
        for f in vehicles:
            x[i,i,f].setAttr('UB', 0.0)
            tau[i,i] = 0.0

    for i in departureNodes:
        for j in nodes[1:-1]:
               for k in arrivalNodes:
                    if [i,j,k] not in P:
                        for f in vehicles:
                            y[i,j,k,f].setAttr('UB', 0.0)
    for i in nodes[1:-1]:
        for f in vehicles:
                p[i,i,f].setAttr('UB', 0.0)
    for i in nodes[1:-1]:
        if i not in uav_serviceable:
            for f in vehicles:
                w[i,f].setAttr('UB', 0.0)

#    for f in vehicles:
#        x[startDepot, endDepot, f].UB = 0.0
#   ==============================================================================
#   ==============================================================================
#   OBJECTIVE
#   ==============================================================================
#   ==============================================================================
    if  min_totalDistance:
        modelVRPD.NumObj = 2
#        Objective min completion time
        modelVRPD.setParam('ObjNumber', 0)
        modelVRPD.ObjNPriority = 2
        z.ObjN = 1
#        for f in vehicles:
#            t[endDepot,f].ObjN = 1
#        Objective min total distance of truck
        modelVRPD.setParam('ObjNumber', 1)
        modelVRPD.ObjNPriority = 1
        modelVRPD.ObjNWeight = 1
        for i in departureNodes:
            for j in arrivalNodes:
                for f in vehicles:
                    x[i,j,f].ObjN = tau[i,j]
    else:
        modelVRPD.setObjective(z)
#        modelVRPD.setObjective(t.sum(customers+1,'*'))
#    + quicksum(0.01 * tau[i,j] * x[i,j,f] for i in departureNodes for j in arrivalNodes for f in vehicles if i != j)
#    + quicksum(0.001 * (tau_d[i,j] + tau_d[j,k]) * y[i,j,k,f] for i in departureNodes for j in nodes[1:-1] for k in arrivalNodes for f in vehicles if [i,j,k] in P)
#    )

#   ==============================================================================
#   ==============================================================================
#   CONSTRAINTS
#   ==============================================================================
#   ==============================================================================

#   DEMAND 7
    modelVRPD.addConstrs(q.sum(j,'*') + w.sum(j,'*')  == 1.0 for j in nodes[1:-1])
    coordconstr = []
    for f in vehicles:
        #   MAXIMUM COMPLETION TIME
        modelVRPD.addConstr(t[endDepot,f] <= z)
        for h in departureNodes:
            for k in arrivalNodes:
                if h != k:
#   RESSOURCE TIME, IF TRUCK VISITS NODE 1
                    modelVRPD.addConstr(t[k,f] >= t[h,f] + tau[h,k] * x[h,k,f]
                        + sl * y.sum(k,'*','*',f)
                        + sr * y.sum('*','*',k,f)
                        - (bigM - tau_d[h,endDepot]) * (1.0 - x[h,k,f]))
#   RESSOURCE TIME; IF DRONE VISITS NODE 2
                    if k in uav_serviceable:
                        modelVRPD.addConstr(t[k,f] >= t[h,f]+float(tau_d[h,k]) * y.sum(h,k,'*',f)
                        - (bigM - tau_d[h,endDepot])*(1.0 - y.sum(h,k,'*',f)))

                    if h in uav_serviceable:
                        modelVRPD.addConstr(t[k,f] >= t[h,f] + (tau_d[h,k] + sr) * y.sum('*',h,k,f)
                        - (bigM - tau_d[h,endDepot]) * (1.0 - y.sum('*',h,k,f)))
#                    for j in uav_serviceable:
#                        modelVRPD.addConstr(t[k,f] >= t[h,f] + (tau_d[h,j] + tau_d[j,k] + sl + sr) * y[h,j,k,f] - (bigM - tau_d[h,endDepot]) * (1.0 - y[h,j,k,f]))

#   LOWER BOUND FOR EACH NODE VI
                    if lb:
                        if h in nodes[1:-1]:
                            if k in nodes[1:-1]:
                                modelVRPD.addConstr(t[k,f] >= t[h,f] + tau[h,k] * p[h,k,f] - (bigM-tau_d[h,endDepot])*(1.0-p[h,k,f]))
            if lb:
                if h in nodes[1:-1]:
                    i = h
#    LOWER BOUND FOR COMPLETION TIME OF EACH VEHICLE VI
                    modelVRPD.addConstr(t[endDepot,f] >=  t[i,f] + quicksum(x[i,j,f] * (tau[i,j] + tau[j,endDepot]) for j in arrivalNodes if i!=j)
                            + quicksum(y[j,i,k,f] * (tau_d[i,k] + tau[k,endDepot]) for j in departureNodes for k in arrivalNodes))
#    LOWER BOUND FOR EACH NODE VI
                    modelVRPD.addConstr(t[i,f] >= quicksum(x[j,i,f] * (tau[startDepot,j] + tau[j,i]) for j in departureNodes if i!=j)
                            + quicksum(y[j,i,k,f]*(tau[startDepot,j] + float(tau_d[j,i])) for j in departureNodes for k in arrivalNodes))
#    UPPER BOUND FOR EACH NODE VI
                    modelVRPD.addConstr(t[h,f] <= q[h,f] * (bigM - tau[h,endDepot]) + w[h,f] * (bigM - float(tau_d[h,endDepot])))

#   MAXIMUM RANGE OF DRONE 3
        modelVRPD.addConstrs(t[k,f] - (t[j,f] - tau_d[i,j]) - (e + bigM * (1-y[i,j,k,f])) <= 0.0
                        for k in nodes[1:-1] for j in uav_serviceable for i in departureNodes if [i,j,k] in P)

#   COORDINATION OF LAUNCHES AND RENDEZVOUS 4
        for i in nodes[1:-1]:
            for l in nodes[1:-1]:
                if i != l:
                    if not coord_relax:
                        for k in nodes[1:-1]:
                            if k!=l and k!=i:
                                list_var = []
                                for m in nodes[1:-1]:
                                    if m != i and m != k:
                                        for n in arrivalNodes:
                                            if n != i and n != k:
                                                list_var.append((1.0, y[l,m,n,f]))
                                expr1 = LinExpr(list_var)
                                coordconstr.append(modelVRPD.addConstr(expr1 <= 3.0 - quicksum(y[i,j,k,f] for j in uav_serviceable if j != l) - p[i,l,f] - p[l,k,f]))
#   TO DEPOT 5
                    list_var = []
                    for m in nodes[1:-1]:
                        if m != i:
                            for n in arrivalNodes:
                                if n != i:
                                    list_var.append((1.0, y[l,m,n,f]))
                    expr = LinExpr(list_var)
                    modelVRPD.addConstr(expr <= 2.0 - quicksum(y[i,j,endDepot,f] for j in uav_serviceable if j != l) - p[i,l,f])

##   FROM DEPOT 5
                    modelVRPD.addConstr(expr <= 2.0 - quicksum(y[startDepot,j,i,f] for j in uav_serviceable if j != l) - p[l,i,f])

#   FLIGHT FROM DEPOT TO DEPOT 5
            if i in uav_serviceable:
                modelVRPD.addConstr(w[i,f] + quicksum(y[startDepot,j,endDepot,f] for j in uav_serviceable if i!=j) <= 1.0)

#   VISITING 6
        for i in nodes:
            if i in arrivalNodes:
                modelVRPD.addConstr(x.sum('*',i,f) - q[i,f] == 0.0)
            if i in departureNodes:
                modelVRPD.addConstr(x.sum(i,'*',f) - q[i,f] == 0.0)
            if i in uav_serviceable:
                modelVRPD.addConstr(y.sum('*',i,'*',f) - w[i,f] == 0.0)

#   ORDERING OF NODES 9
        for idx, i in enumerate(nodes[1:-2]):
            for j in nodes[idx+2:-1]:
                modelVRPD.addConstr(b[i,j,f] <= q[i,f])
                modelVRPD.addConstr(b[i,j,f] <= q[j,f])
                modelVRPD.addConstr(b[i,j,f] >= q[i,f] + q[j,f] - 1)
                modelVRPD.addConstr(p[i,j,f] + p[j,i,f] == b[i,j,f], name = 'Test')
        for i in nodes[1:-1]:
            for j in nodes[1:-1]:
                if i!=j:
#                    modelVRPD.addConstr(u[j,f]-u[i,f] - (1.0 - q[i,f]) * (customers-1)<= (customers) * p[i,j,f])
#                    modelVRPD.addConstr(u[j,f]-u[i,f] - (1.0) * (customers - 1) <= (customers - 1) * p[i,j,f])
#                    modelVRPD.addConstr(u[j,f] - u[i,f] - p[i,j,f] * (customers - 1) >= - (customers - 1.0))
                    modelVRPD.addConstr(u[i,f] - u[j,f] <= (customers - 1) - customers * p[i,j,f])
#                    modelVRPD.addConstr(q[i,f] + q[j,f] - p[i,j,f] - p[j,i,f] <= 1.0)
#                    if j < customers+1:
#                        modelVRPD.addConstr(u[i,f]-u[j,f] + customers * x[i,j,f] + (customers-2) * x[j,i,f] <= (customers - q[j,f])) #   LIFTED MTZ -> DL 8
        for i in nodes[1:-1]:
#            if i == 0:
#                for j in nodes[1:-1]:
#                    modelVRPD.addConstr(u[i,f]-u[j,f] + customers * x[i,j,f] <= (customers - q[j,f])) #   LIFTED MTZ -> DL 8
#            else:
                for j in nodes[1:-1]:
                        modelVRPD.addConstr(u[i,f]-u[j,f] + customers * x[i,j,f] + (customers-2) * x[j,i,f] <= (customers - q[j,f])) #   LIFTED MTZ -> DL 8
#   10
        for i in departureNodes:
            modelVRPD.addConstr(y.sum(i,'*','*',f) - q[i,f] <= 0.0) #   START NODE OF DRONE HAS TO BE VISITED BY TRUCK 10
        for i in arrivalNodes:
            modelVRPD.addConstr(y.sum('*','*',i,f) - q[i,f] <= 0.0) #   END NODE OF DRONE HAS TO BE VISITED BY TRUCK 10

#    ==============================================================================
#    ==============================================================================
#    VALID INEQUALITIES
#    ==============================================================================
#    ==============================================================================
    if vi:
#    SYMMETRY BREAKING
#        modelVRPD.addConstrs(t[endDepot,f-1] <= t[endDepot,f] for f in vehicles[1:])
        modelVRPD.addConstrs(quicksum(q[i,f] + w[i,f] for i in nodes[1:-1]) >= quicksum(q[i,f+1] + w[i,f+1] for i in nodes[1:-1]) for f in vehicles[:-1])
        for f in vehicles:
            for i in nodes[1:-1]:
                modelVRPD.addConstr(y.sum(i,'*','*',f) == y.sum('*','*',i,f))
#            for i in nodes[1:-1]:
#                modelVRPD.addConstr(u[i,f] >= q[i,f])
#                modelVRPD.addConstr(u[i,f] <= q[i,f] * customers)
#                modelVRPD.addConstr(u[i,f] <= q.sum('*',f))
#                modelVRPD.addConstr(u[i,f] <= (customers) - (customers - 1) * x[startDepot,i,f])
#                modelVRPD.addConstr(u[i,f] >= (customers) - (customers - 1) * x[i,endDepot,f])
#       START NODE OF DRONE HAS TO BE BY VISITED TRUCK BEFORE LANDING NODE
            modelVRPD.addConstrs(p[i,k,f] >= y.sum(i,'*',k,f) for i in nodes[1:-1] for k in nodes[1:-1] if i!=k)

#                1) MAXIMUM DRONE FLIGHTS
            if validInequalities[0]:
                modelVRPD.addConstr(w.sum('*',f) - quicksum(q[i,f] for i in arrivalNodes) <= 0.0) #FT

#                2) 3CLIQUE-CUTS
            if validInequalities[1]:
#                for i in nodes[1:-1]:
#                    for j in nodes[1:-1]:
#                        if i != j:
#                            modelVRPD.addConstr(quicksum(x[h,i,f] + x[h,j,f] for h in departureNodes if h != i and h != j)  + quicksum(x[i,h,f] + x[j,h,f] for h in arrivalNodes if h != i and h != j) >= q[i,f] + q[j,f])
#                modelVRPD.addConstrs(x[i,j,f] + x[j,i,f] <= 1.0 for i in nodes[1:-1] for j in nodes[1:-1])
#                modelVRPD.addConstrs(x[i,j,f] + x[j,i,f] + x[i,k,f] + x[k,i,f] + x[j,k,f] + x[k,j,f] <= 2.0
#                                for i in nodes[1:-3] for j in nodes[(i+1):-2] for k in nodes[(j+1):-1])
                modelVRPD.addConstrs(q[startDepot,f] >= q[j,f] for j in nodes[1:-1])
#                modelVRPD.addConstrs(q[j,f] <= quicksum(x[startDepot,i,f] for i in nodes[1:-1]) for j in nodes[1:-1])
                modelVRPD.addConstrs(x[startDepot,endDepot,f] <= 1 - q[j,f] for j in nodes[1:-1])
#                3) TRINAGLE CUTS
            if validInequalities[2]:
                for i in nodes[1:-1]:
                    for j in nodes[1:-1]:
                        if i != j:
                            for k in nodes[1:-1]:
                                if j!=k and i!=k:
                                    modelVRPD.addConstr(p[i,j,f] + p[j,k,f] + p[k,i,f] <= 2.0, name ='VI_5_%s_%s_%s_%s' % (i,j,k,f))

#               4)
            if validInequalities[3]:
                for i in nodes[1:-1]:
                    for j in nodes[1:-1]:
                        if i!=j:
                            modelVRPD.addConstr(p[i,j,f] - x[i,j,f] >= 0.0) #Änderung
                            modelVRPD.addConstr(p[i,j,f] + x[j,i,f] <= 1.0)
#    ==============================================================================
#    ==============================================================================
#    VARIABLE BOUNDS
#    ==============================================================================
#    ==============================================================================
    for f in vehicles:
#        q[startDepot,f].LB = 1
#        q[endDepot,f].LB = 1
        t[startDepot,f].setAttr("ub", 0.0)

    for constr in coordconstr:
        constr.setAttr('Lazy', 1)
    return modelVRPD

#######################################################################################################################################################################
#######################################################################################################################################################################
#######################################################################################################################################################################

def changeObjective_to_minDistance(model, vehicles, nodes, velocities):
    tau = velocities[0]
    endDepot = nodes[-1]
    model.getVarByName('z').Obj = 0
    for i in nodes[:-1]:
        for j in nodes[1:]:
            if i != j:
                for f in vehicles:
                    var = model.getVarByName('x[' + str(i) + ',' + str(j) + ',' + str(f) + ']')
                    var.Obj = tau[i,j]
    model.addConstrs(model.getVarByName('t[' + str(endDepot) + ',' + str(f) + ']') <= model.ObjVal + 0.1 for f in vehicles)
    return model

#######################################################################################################################################################################
#######################################################################################################################################################################
#######################################################################################################################################################################

def buildModel_TSPD(customers, sl, sr, e, M, nodes, uav_serviceable, tau, tau_d, P):

    tsp_d = Model('TSPD')

    x = {}
    for i in nodes[:-1]:
        for j in nodes[1:]:
            if i != j:
                x[i, j] = tsp_d.addVar(vtype=GRB.BINARY,  name='x_%s_%s'
                                       % (i, j))

    y = {}
    for i in nodes[:-1]:
        for j in nodes[1:-1]:
            if j != i:
                for k in nodes[1:]:
                    if [i, j, k] in P:
                        y[i, j, k] = tsp_d.addVar(vtype=GRB.BINARY,
                                                  name='y_%s_%s_%s' %
                                                  (i, j, k))
    u = {}
    for i in nodes[1:]:
        u[i] = tsp_d.addVar(vtype=GRB.INTEGER,  lb=1,  name='u_%s' %
                            (i))

    t = {}
    for i in nodes:
        t[i] = tsp_d.addVar(vtype=GRB.CONTINUOUS,  name='t_%s' % (i))

    t_d = {}
    for i in nodes:
        t_d[i] = tsp_d.addVar(vtype=GRB.CONTINUOUS,  name='t_d_%s' %
                              (i))

    p = {}
    for i in nodes[:-1]:
        for j in nodes:
            if i != j:
                p[i, j] = tsp_d.addVar(vtype=GRB.BINARY,  name='p_%s_%s'
                                       % (i, j))

    z = tsp_d.addVar(vtype=GRB.CONTINUOUS,  name='z')

    tsp_d.update()

    # CONSTRAINTS

    for j in nodes[1:-1]:
        tsp_d.addConstr(quicksum(x[i, j] for i in nodes[:-1] if i != j)
                        + quicksum(y[i, j, k] for k in nodes[1:] for i
                                   in nodes[:-1] if i != j and [i, j, k]
                                   in P) == 1,  name='2_%s' % j)

    tsp_d.addConstr(quicksum(x[0, j] for j in nodes[1:]) == 1, name='3')

    tsp_d.addConstr(quicksum(x[j, customers+1] for j in nodes[:-1]) == 1,
                    name='4')

    for i in nodes[1:-1]:
        for j in nodes[1:]:
            if j != i:
                tsp_d.addConstr(u[i]-u[j]+1 <= (customers+2)*(1-x[i, j]),
                                name='5_%s_%s' % (i, j))

    for j in nodes[1:-1]:
        tsp_d.addConstr(quicksum(x[i, j] for i in nodes[:-1] if i != j) ==
        quicksum(x[j, k] for k in nodes[1:] if k != j),  name='6_%s' % (j))


    for i in nodes[:-1]:
        tsp_d.addConstr(quicksum(y[i, j, k] for j in nodes[1:-1] for k in
                                 nodes[1:] if j != i and [i, j, k] in P)
                        <= 1,  name='7_%s' % (i))

    for k in nodes[1:]:
        tsp_d.addConstr(quicksum(y[i, j, k] for i in nodes[:-1] for j in
                                 nodes[1:-1] if i != k and [i, j, k] in P)
                        <= 1,  name='8_%s' % (i))

    for i in nodes[1:-1]:
        for j in nodes[1:-1]:
            if i != j:
                for k in nodes[1:]:
                    if [i, j, k] in P:
                        tsp_d.addConstr(2*y[i, j, k] <= quicksum(x[h, i]
                                                                 for h in
                                                                 nodes[:-1]
                                                                 if h != i)
                        + quicksum(x[l, k] for l in nodes[1:-1] if l != k),
                                        name='9_%s_%s_%s' % (i, j, k))

    for j in nodes[1:-1]:
        for k in nodes[1:]:
            if [0, j, k] in P:
                tsp_d.addConstr(y[0, j, k] <= quicksum(x[h, k] for h in
                                                       nodes[:-1] if h != k),
                                name='10_%s_%s' % (j, k))

    for i in nodes[1:-1]:
        for k in nodes[1:]:
            if k != i:
                tsp_d.addConstr(u[k]-u[i] >= 1-(customers+2)*(1-quicksum(
                            y[i, j, k] for j in nodes[1:-1] if [i, j, k]
                            in P)),  name='11_%s_%s' % (i, k))

    for i in nodes[1:-1]:
        tsp_d.addConstr(t_d[i] >= t[i] - M*(1- quicksum(y[i, j, k] for j
                                                        in nodes[1:-1]
                                                        for k in nodes[1:]
                                                        if j != i and
                                                        [i, j, k] in P)),
                        name='12_%s' % (i))

    for i in nodes[1:-1]:
        tsp_d.addConstr(t_d[i] <= t[i] + M*(1- quicksum(y[i, j, k] for j
                                                        in nodes[1:-1] for k
                                                        in nodes[1:] if j != i
                                                        and [i, j, k] in P)),
                        name='13_%s' % (i))

    for k in nodes[1:]:
        tsp_d.addConstr(t_d[k] >= t[k] - M*(1- quicksum(y[i, j, k] for j
                                                        in nodes[1:-1] for i
                                                        in nodes[:-1] if i != k
                                                        and [i, j, k] in P)),
                        name='14_%s' % (k))

    for k in nodes[1:]:
        tsp_d.addConstr(t_d[k] <= t[k] + M*(1- quicksum(y[i, j, k] for j
                                                        in nodes[1:-1] for i
                                                        in nodes[:-1] if i != k
                                                        and [i, j, k] in P)),
                        name='15_%s' % (k))

    for h in nodes[:-1]:
        for k in nodes[1:]:
            if k != h:
                tsp_d.addConstr(t[k] >= t[h] + tau[h, k] + sl
                                *quicksum(y[k, l, m] for m in nodes[1:] for l
                                         in nodes[1:-1] if [k, l, m] in P and
                                         l != k) + sr*quicksum(y[i, j, k] for j
                                                               in nodes[1:-1] for i
                                                               in nodes[:-1]
                                                               if [i, j, k] in P
                                                               and i != k) -
                                M*(1-x[h, k]),  name='16_%s_%s' % (h, k))

    for j in uav_serviceable:
        for i in nodes[:-1]:
            if i != j:
                tsp_d.addConstr(t_d[j] >= t_d[i]+tau_d[i, j] -
                                M*(1-quicksum(y[i, j, k] for k in nodes[1:]
                                              if [i, j, k] in P)),
                                name='17_%s_%s' % (j, i))

    for j in uav_serviceable:
        for k in nodes[1:]:
            if k != j:
                tsp_d.addConstr(t_d[k] >= t_d[j]+tau_d[j, k]+sr -
                                M*(1- quicksum(y[i, j, k] for i in nodes[:-1]
                                               if [i, j, k] in P)),
                                name='18_%s_%s' % (j, k))

    for k in nodes[1:]:
        for j in nodes[1:1]:
            if j != k:
                for i in nodes[:-1]:
                    if [i, j, k] in P:
                        tsp_d.addConstr(t_d[k] - (t_d[j] - tau_d[i, j]) <= e + M
                                        *(1-y[i, j, k]),
                                        name='19_%s_%s_%s' % (k, j, i))

    for i in nodes[1:-1]:
        for j in nodes[1:-1]:
            if j != i:
                tsp_d.addConstr(u[i]-u[j] >= 1-(customers+2)*p[i, j],
                                name='20_%s_%s' % (i, j))

    for i in nodes[1:-1]:
        for j in nodes[1:-1]:
            if j != i:
                tsp_d.addConstr(u[i]-u[j] <= - 1 + (customers+2)*(1-p[i, j]),
                                name='21_%s_%s' % (i, j))

    for i in nodes[1:-1]:
        for j in nodes[1:-1]:
            if j != i:
                tsp_d.addConstr(p[i, j] + p[j, i] == 1,
                                name='22_%s_%s' % (i, j))

    for i in nodes[:-1]:
        for k in nodes[1:]:
            if k != i:
                for l in nodes[1:-1]:
                    if l != i and l !=k:
                        tsp_d.addConstr(t_d[l] >= t_d[k] -
                                        M*(3- quicksum(y[i, j, k] for j in nodes[1:-1]
                                                       if j != l and [i, j, k] in P) -
                                           quicksum(y[l, m, n] for n in nodes[1:] for m
                                                    in nodes[1:-1] if n != i and n != k
                                                    and [l, m, n] in P and m != i and
                                                    m != k and m != l) - p[i, l]),
                                        name='23_%s_%s_%s' % (i, k, l))

    tsp_d.addConstr(t[0] == 0, name='24')
    tsp_d.addConstr(t_d[0] == 0, name='25')

    for j in nodes[1:-1]:
        tsp_d.addConstr(p[0, j] == 1, name='26_%s' % (j))

    tsp_d.addConstr(t[customers+1] <= z, name='fin')
    tsp_d.addConstr(t_d[customers+1] <= z, name='fin_d')

    for i in nodes[1:]:
        tsp_d.addConstr(u[i] <= customers +2, name='29_%s' % (i))

    tsp_d.update()
    tsp_d.setObjective(z)

    return tsp_d

#def buildModel_TSP()