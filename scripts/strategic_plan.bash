#!/bin/bash
rosservice call /rosplan_interface_strategic_control/decompose_problem
rosservice call /strategic_problem_interface/problem_generation_server;
rosservice call /strategic_planner_interface/planning_server;
rosservice call /strategic_parsing_interface/parse_plan;
rosservice call /strategic_plan_dispatch/dispatch_plan;
