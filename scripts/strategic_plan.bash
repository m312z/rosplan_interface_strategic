#!/bin/bash

param1="knowledge:
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'strategic_planning'
  function_value: 0.0";
rosservice call /kcl_rosplan/update_knowledge_base_array "update_type: 0
$param1"

param2="knowledge:
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'complete'
  values:
  - {key: 'i', value: 'i01'}
  - {key: 'wp', value: 'wp1'}
  function_value: 0.0
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'complete'
  values:
  - {key: 'i', value: 'i02'}
  - {key: 'wp', value: 'wp2'}
  function_value: 0.0
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'complete'
  values:
  - {key: 'i', value: 'i03'}
  - {key: 'wp', value: 'wp3'}
  function_value: 0.0";
rosservice call /kcl_rosplan/update_knowledge_base_array "update_type: 1
$param2"


echo "Calling strategic problem generator.";
rosservice call /strategic_problem_interface/problem_generation_server;
echo "Calling strategic planner interface.";
rosservice call /strategic_planner_interface/planning_server;
echo "Calling strategic plan parser.";
rosservice call /strategic_parsing_interface/parse_plan;

echo "Resetting goals for execution.";
rosservice call /kcl_rosplan/update_knowledge_base_array "update_type: 3
$param2"


echo "Calling strategic plan dispatcher.";
rosservice call /strategic_plan_dispatch/dispatch_plan;
echo "Finished!";
