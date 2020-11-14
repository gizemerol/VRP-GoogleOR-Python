# Vehicle Routing Problem with Service Time
The VRP is one of the most attractive optimization problems. 
Many different variants have studied in the literature such as capacitated VRP, 
VRP with time windows etc. 

I have solved this problem implementing Google OR-Tools linear solver. 
The inputs of this problem are vehicles, orders and distance matrix between orders.
Input data is given in the "input.json" file in detail.

The aim of the problem is to find the minimum total total distance. Thus, we 
need to decide optimum order assignment to each vehicle considering problem
constraints.

Problem definition, objective and constraints are given in the following part
 in detail:

#### Problem definition
* Each vehicle starts a different depot.
* Vehicles don't have to return to their initial depot.
* Each order has different service time.

#### Objective
* The aim  is to find optimum routes that minimize the total distance.


#### Constraints
* Each order should be assigned to a vehicle.
* All subtours should be eliminated.
* Flow conservation should be preserved.


#### Output
* Total delivery time of each order
* Consecutive assigned orders to each vehicle 



