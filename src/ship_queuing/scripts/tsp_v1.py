#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
import math
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


class WPRouting():
    def __init__(self):
        self.wpsub = rospy.Subscriber('WPs',Float32MultiArray,self.wp_callback)
        self.wpsub

        self.owppub = rospy.Publisher('OrderedWPs',Float32MultiArray,queue_size=10)
        self.owplistpub = rospy.Publisher('OrderedWPsList',Float32MultiArray,queue_size=10)

        timer_period = rospy.Duration(1) # seconds
        self.timer = rospy.Timer(timer_period, self.time_callback)

        self.flagsub = rospy.Subscriber('Flag',Int32,self.flag_callback)
        self.flagsub

        self.WPS = 0  # 정렬 전 WPs
        self.WPS_order = 0  # TSP 정렬 후 WPs 순서 
        self.WPS_this_order = 0 # 현재 가야하는 WP
        self.visited_WPS_order_list = []
        self.i = 0
        self.flag_num = 0


    def wp_callback(self, msg):
        # rospy.loginfo(rospy.get_caller_id() + 'WPS %s', msg.data)
        origin_wps = msg.data  # 받은 WPS 그대로 
        cut_wps = origin_wps  # flag 받으면 지나간 WP 자르고 적용, 일단 wps 안바꾸고 자르는 방법으로 
        # visit_checker = 0
        # for i in range(len(origin_wps)):
        #     for j in range(len(self.visited_WPS_order_list)):
        #         if i == self.visited_WPS_order_list[j]*2:
        #             visit_checker = 1
        #         elif i == self.visited_WPS_order_list[j]*2+1:
        #             visit_checker = 1
        #         else:
        #             pass

        #     if visit_checker == 0:
        #         cut_wps.append(origin_wps[i])
        #     else:
        #         visit_checker = 0


        wps = []
        wp_size = int(len(cut_wps)/2)
        for i in range(wp_size):
            wps.append((cut_wps[2*i],cut_wps[2*i+1]))
        self.WPS = wps

        """TSP START"""
        tsp_solver = tsp_2d
        data = tsp_solver.create_data_model(wps)
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
            self.WPS_this_order = self.WPS_order[0][self.flag_num + 1]
        """TSP END"""

    def time_callback(self, timedata):
        rospy.loginfo("Time: %s" % rospy.get_time())

        owps = []
        single_wp = []
        msg = Float32MultiArray()
        msg_list = Float32MultiArray()
        if self.WPS_order == 0:
            self.owppub.publish(msg)
            self.owplistpub.publish(msg_list)
            print("WPs Not Received")
        else:
            # owps: whole WPs list / single_wp: only next WP
            for i in range(len(self.WPS)):
                owps.append(self.WPS[self.WPS_order[0][i]][0])
                owps.append(self.WPS[self.WPS_order[0][i]][1])

            single_wp.append(self.WPS[self.WPS_order[0][self.flag_num + 1]][0])
            single_wp.append(self.WPS[self.WPS_order[0][self.flag_num + 1]][1])


            msg.data = single_wp
            msg_list.data = owps
            self.owppub.publish(msg)           
            self.owplistpub.publish(msg_list)
            print("Ordered WPs:", owps)
            print("Next WP    :", single_wp)
            print("====================================================================")

        self.i += 1


    def flag_callback(self, data):
        rospy.loginfo("Flag Received")
        self.visited_WPS_order_list.append(self.WPS_this_order)
        self.flag_num = self.flag_num + 1


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

