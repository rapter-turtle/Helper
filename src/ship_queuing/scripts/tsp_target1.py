#!/usr/bin/env python3

import rospy
import sys
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import math
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


## depot_x,depot_y,depot_vx, depot_vy, number of IDs, (ID, ID_x, ID_y , ID_vx, ID_vy)

class WPRouting():
    def __init__(self):
        self.wpsub = rospy.Subscriber('global_track',Float32MultiArray,self.wp_callback)
        self.flagsub = rospy.Subscriber('usv_flag',Int32,self.flag_callback)
        
        self.owppub = rospy.Publisher('wp_id',Float32,queue_size=10)
        self.flagpub = rospy.Publisher('flag_3',Int32,queue_size =10)


        self.WPS_order = []  # TSP 정렬 후 WPs 순서 
        self.WPS_this_order = 0 # 현재 가야하는 WP
        self.visited_WPS_ids_list = []
        self.flag = 0
        self.sent = 0
        self.msg_pub_1= Float32()
        self.msg_pub_2 = Int32()



    def wp_callback(self, msg):
        print(self.flag)
        if self.flag == 2 and self.sent == 0:
            origin_wps = msg.data  
            wps_ids_xy = [(0,origin_wps[0], origin_wps[1])]
            wps_xy = [(origin_wps[0], origin_wps[1])]
            wp_size = int(msg.data[2])
            for i in range(wp_size):
                for j in self.visited_WPS_ids_list:
                    if j == int(origin_wps[5*i+3]):
                        break
                else:    
                    wps_ids_xy.append((origin_wps[5*i+3],origin_wps[5*i+4],origin_wps[5*i+5]))
                    wps_xy.append((origin_wps[5*i+4],origin_wps[5*i+5]))

            """TSP START"""
            tsp_solver = tsp_2d
            data = tsp_solver.create_data_model(wps_xy)
            manager = pywrapcp.RoutingIndexManager(len(data['locations']),
                                                data['num_vehicles'], data['depot'])
            routing = pywrapcp.RoutingModel(manager)
            distance_matrix = tsp_solver.compute_euclidean_distance_matrix(data['locations'])

            def distance_callback(from_index, to_index):
                from_node = manager.IndexToNode(from_index)
                to_node = manager.IndexToNode(to_index)
                return distance_matrix[from_node][to_node]

            transit_callback_index = routing.RegisterTransitCallback(distance_callback)
            routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
            search_parameters = pywrapcp.DefaultRoutingSearchParameters()
            search_parameters.first_solution_strategy = (
                routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
            solution = routing.SolveWithParameters(search_parameters)

            if solution:
                # tsp_solver.print_solution(manager, routing, solution)
                routes = tsp_solver.get_routes(solution, routing, manager)
                self.WPS_order = routes
                # if second node is closer than the last node
                if distance_matrix[0][self.WPS_order[0][1]] <= distance_matrix[0][self.WPS_order[0][-2]]:
                    self.WPS_this_order = self.WPS_order[0][1]
                else:
                    self.WPS_this_order = self.WPS_order[0][-2]

            """TSP END"""


            print(type(wps_ids_xy[self.WPS_this_order][0]))
            for i in self.WPS_order[0]:
                print(wps_ids_xy[i][0])
            self.msg_pub_1.data = wps_ids_xy[self.WPS_this_order][0]
            self.msg_pub_2.data = 1

            self.owppub.publish(self.msg_pub_1)
            self.flagpub.publish(self.msg_pub_2)
            self.visited_WPS_ids_list.append(wps_ids_xy[self.WPS_this_order][0])
            self.flag = 0
            self.sent = 1

        elif self.flag == 2:
            self.owppub.publish(self.msg_pub_1)
            self.flagpub.publish(self.msg_pub_2)

        elif self.flag >= 3:
            self.sent = 0
        

    def flag_callback(self, msg):
        self.flag = msg.data


class tsp_2d:
    def create_data_model(waypoints):
        """Stores the data for the problem."""
        data = {}
        # Locations in block units
        data['locations'] = waypoints
        data['num_vehicles'] = 1
        data['depot'] = 0
        return data
        # [END data_model]


    # [START distance_callback]
    def compute_euclidean_distance_matrix(locations):
        """Creates callback to return distance between points."""
        distances = {}
        for from_counter, from_node in enumerate(locations):
            distances[from_counter] = {}
            for to_counter, to_node in enumerate(locations):
                if from_counter == to_counter:
                    distances[from_counter][to_counter] = 0
                else:
                    # Euclidean distance
                    distances[from_counter][to_counter] = (int(
                        math.hypot((from_node[0] - to_node[0]),
                                (from_node[1] - to_node[1]))))
        return distances
        # [END distance_callback]


    # [START solution_printer]
    def print_solution(manager, routing, solution):
        """Prints solution on console."""
        print('Objective: {}'.format(solution.ObjectiveValue()))
        index = routing.Start(0)
        plan_output = 'Route:\n'
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += ' {} ->'.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
        plan_output += ' {}\n'.format(manager.IndexToNode(index))
        print(plan_output)
        plan_output += 'Objective: {}m\n'.format(route_distance)
        # [END solution_printer]       
    
    def get_routes(solution, routing, manager):
        """Get vehicle routes from a solution and store them in an array."""
        # Get vehicle routes and store them in a two dimensional array whose
        # i,j entry is the jth location visited by vehicle i along its route.
        routes = []
        for route_nbr in range(routing.vehicles()):
            index = routing.Start(route_nbr)
            route = [manager.IndexToNode(index)]
            while not routing.IsEnd(index):
                index = solution.Value(routing.NextVar(index))
                route.append(manager.IndexToNode(index))
            routes.append(route)
        return routes

def main():
    
    rospy.init_node('tsp_v1', anonymous=True)
    WPR = WPRouting()

    try:
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
