"""
VRP with different ending nodes
Authors: Gizem Erol  2020
"""

#Importing libraries
from __future__ import print_function
from ortools.linear_solver import pywraplp
import numpy as np
import json

#Loading json file and getting optimization results
def getInputDataSendSolution():
    with open("input.json") as write_file:
        data = json.load(write_file)
    solution = vehicleRouting(data) #Optimization model function
    return solution

#Optimization model & output
def vehicleRouting(data):
    # Inputs
    vehicle = data['vehicles']
    vehicle_count = len(vehicle)
    orders = data['jobs']
    order_count = len(orders) + len(vehicle)
    matrix = data['matrix']

    #Sets
    vehicles = range(0, vehicle_count)  # vehicle (beginning) nodes
    order_nodes = range(vehicle_count, order_count) #only order nodes
    vehicle_order_nodes_together = range(0, order_count)  #vehicles and orders nodes together
    ending_nodes = range(order_count, vehicle_count + order_count) #dummy ending nodes of the vehicles
    order_ending_nodes_together = range(vehicle_count,
                              vehicle_count + order_count)  # orders and dummy ending nodes together
    all_nodes_together = range(0, vehicle_count + order_count) #all nodes

    # Model
    solver = pywraplp.Solver('Routing Problem', pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)

    #Decision Variables
    # Assignment Decision Variables
    assignment_variables = [[[solver.IntVar(0, 1, 'assignment_variables[%i,%i,%i]' % (i, j, k)) for k in vehicles] for j in all_nodes_together] for i in
               all_nodes_together]

    # Subtour Decision Variables
    subtour_variables = [[solver.IntVar(0, solver.infinity(), 'subtour_variables[%i,%i]' % (j, k)) for k in vehicles] for j in
               vehicle_order_nodes_together]

    # Objective Function
    # Minimizing total distance
    objective = solver.Objective()
    objective.SetMinimization()
    for i in vehicle_order_nodes_together:
        for j in vehicle_order_nodes_together:
            for k in vehicles:
                objective.SetCoefficient(assignment_variables[i][j][k], matrix[i][j])

    # Decision Variable Constraints: Eliminating the decision variables that take 0 value to reduce processing time of the model
    # Since we have different beginning and ending nodes, the beginning nodes cannot be assigned as ending nodes.
    c = solver.Constraint(0, 0)
    for i in all_nodes_together:
        for j in vehicles:
            for k in vehicles:
                c.SetCoefficient(assignment_variables[i][j][k], 1)

    # The ending nodes cannot be assigned as beginning nodes.
    c = solver.Constraint(0, 0)
    for i in ending_nodes:
        for j in all_nodes_together:
            for k in vehicles:
                c.SetCoefficient(assignment_variables[i][j][k], 1)

    # If vehicle index does not match with beginning node index, they cannot be assigned.
    c = solver.Constraint(0, 0)
    for i in vehicles:
        for j in all_nodes_together:
            for k in vehicles:
                if i != k:
                    c.SetCoefficient(assignment_variables[i][j][k], 1)

    # Same index cannot be beginning and ending node simultaneously.
    c = solver.Constraint(0, 0)
    for k in vehicles:
        for j in all_nodes_together:
            for i in all_nodes_together:
                if i == j:
                    c.SetCoefficient(assignment_variables[i][j][k], 1)

    # Problem Constraints: These constraints are necessary because of the nature of the VRP.
    # Subtour Elimination Constraints
    bigM = max(max(matrix))
    for i in vehicle_order_nodes_together:
        for j in vehicle_order_nodes_together:
            for k in vehicles:
                if j > vehicle_count:
                    c = solver.Constraint(-bigM, bigM - matrix[i][j])
                    c.SetCoefficient(subtour_variables[i][k], 1)
                    c.SetCoefficient(subtour_variables[j][k], -1)
                    c.SetCoefficient(assignment_variables[i][j][k], bigM)

    # Each order should be assigned at once.
    for i in order_nodes:
        c = solver.Constraint(1, 1)
        for k in vehicles:
            for j in order_ending_nodes_together:
                c.SetCoefficient(assignment_variables[i][j][k], 1)

    # If a vehicle is used in the optimal solution, its matching beginning node should be assigned.
    for i in vehicles:
        for k in vehicles:
            c = solver.Constraint(0, 1)
            if i == k:
                for j in all_nodes_together:
                    c.SetCoefficient(assignment_variables[i][j][k], 1)

    #Vehicles should be assigned their dummy ending nodes.
    for j in ending_nodes:
        for k in vehicles:
            c = solver.Constraint(0, 1)
            if (k + order_count) == j:
                for i in order_nodes:
                    c.SetCoefficient(assignment_variables[i][j][k], 1)

    #Flow Conservation Constraints
    for k in vehicles:
        for i in vehicles:
            c = solver.Constraint(0, 0)
            for j in all_nodes_together:
                c.SetCoefficient(assignment_variables[i][j][k], 1)

            for j in all_nodes_together:
                c.SetCoefficient(assignment_variables[j][i + order_count][k], -1)

    # Flow Conservation Constraints (If a vehicle visit a node, same vehicle should leave that node)
    for i in order_nodes:
        for k in vehicles:
            c = solver.Constraint(0, 0)
            for j in all_nodes_together:
                c.SetCoefficient(assignment_variables[i][j][k], 1)

            for m in all_nodes_together:
                c.SetCoefficient(assignment_variables[m][i][k], -1)

    #OUTPUT
    status = solver.Solve()
    if status == pywraplp.Solver.OPTIMAL or status == pywraplp.Solver.FEASIBLE:
        output = dict.fromkeys([p + 1 for p in vehicles])
        timeProducts = dict.fromkeys(range(1, len(orders) + 1))
        for k in vehicles:
            last_assignment = []
            for i in vehicle_order_nodes_together:
                for j in all_nodes_together:
                    if assignment_variables[i][j][k].solution_value() == 1:
                        last_assignment.append([i, j])

            #Sorting the orders
            temp = last_assignment.pop(0)
            while len(temp) <= len(last_assignment):
                temp.extend([q[1] for q in last_assignment if q[0] == temp[-1]])
            #Arrival time calculation for each order
            timings = (np.array([matrix[temp[a - 1]][temp[a]] for a in range(1, len(temp))]).cumsum())
            for index, t in enumerate(temp[1:]):
                timeProducts[t - 2] = float(timings[index])
            output[k + 1] = [str(p - 2) for p in temp[1:]]

        timeProducts['vehicleRoutes'] = output
        return timeProducts
#
#Running the model
print(getInputDataSendSolution())

