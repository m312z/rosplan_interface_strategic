#!/bin/bash

mats=(
	'black'
	'blue'
	'green'
	'red'
	'white'
   )

param="knowledge:
- knowledge_type: 0
  instance_type: 'robot'
  instance_name: 'kenny'
  attribute_name: ''
  function_value: 0.0";
for i in $(seq 0 4 )
do
param="$param
- knowledge_type: 0
  instance_type: 'waypoint'
  instance_name: 'wp$i'
  attribute_name: ''
  function_value: 0.0
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'material_at'
  values:
  - {key: 'm', value: '${mats[$i]}'}
  - {key: 'wp', value: 'wp$i'}
  function_value: 0.0
- knowledge_type: 2
  instance_type: ''
  instance_name: ''
  attribute_name: 'carrying'
  values:
  - {key: 'r', value: 'kenny'}
  - {key: 'm', value: '${mats[$i]}'}
  function_value: 0.0";
done;

param="$param
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'robot_at'
  values:
  - {key: 'r', value: 'kenny'}
  - {key: 'wp', value: 'wp0'}
  function_value: 0.0
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'docked'
  values:
  - {key: 'r', value: 'kenny'}
  function_value: 0.0
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'dock_at'
  values:
  - {key: 'wp', value: 'wp0'}
  function_value: 0.0";

rosservice call /kcl_rosplan/update_knowledge_base_array "update_type: 0
$param"
