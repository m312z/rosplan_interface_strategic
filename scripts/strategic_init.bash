#!/bin/bash

mats=(
	'black'
	'blue'
	'green'
	'white'
	'red'
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
  attribute_name: 'not_busy'
  function_value: 0.0
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
  function_value: 0.0
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'not_localised'
  values:
  - {key: 'r', value: 'kenny'}
  function_value: 0.0";

rosservice call /kcl_rosplan/update_knowledge_base_array "update_type: 0
$param"

for i in $(seq 0 2 )
do
param="update_type: 2
duration: $((60+$i*120))
knowledge:
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'material_at'
  values:
  - {key: 'm', value: '${mats[4]}'}
  - {key: 'wp', value: 'wp4'}
  function_value: 0.0";
rosservice call /kcl_rosplan/update_knowledge_base_array "$param";
done;

for i in $(seq 1 3 )
do
param="update_type: 0
duration: $(($i*120))
knowledge:
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'material_at'
  values:
  - {key: 'm', value: '${mats[4]}'}
  - {key: 'wp', value: 'wp4'}
  function_value: 0.0";
rosservice call /kcl_rosplan/update_knowledge_base_array "$param";
done;
