#!/bin/bash
rosservice call /rosplan_interface_strategic_control/decompose_problem
rosservice call /strategic_problem_interface/problem_generation_server;
rosservice call /strategic_planner_interface/planning_server;
rosservice call /strategic_parsing_interface/parse_plan;
param="knowledge:
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'can_tac_goto'
  function_value: 0.0"

rosservice call /kcl_rosplan/update_knowledge_base_array "update_type: 0
$param"
rosservice call /strategic_plan_dispatch/dispatch_plan;