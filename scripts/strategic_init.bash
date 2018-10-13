#!/bin/bash

mats=(
	'black'
	'blue'
	'green'
	'white'
	'red'
   )

type="update_type:
- 0";
param="
knowledge:
- knowledge_type: 0
  instance_type: 'robot'
  instance_name: 'kenny'
  attribute_name: ''
  function_value: 0.0";
for i in $(seq 0 3 )
do
type="$type
- 0
- 0
- 0";
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

type="$type
- 0
- 0
- 0
- 0
- 0";
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

for i in $(seq 1 8 )
do
type="$type
- 0
- 0";
param="$param
- knowledge_type: 1
  initial_time: { secs: $(( $(date +%s) + $i*600)), nsecs: 0}
  instance_type: ''
  instance_name: ''
  attribute_name: 'material_at'
  values:
  - {key: 'm', value: '${mats[4]}'}
  - {key: 'wp', value: 'wp0'}
  is_negative: False
- knowledge_type: 1
  initial_time: { secs: $(( $(date +%s) + 60 + $i*600)), nsecs: 0}
  instance_type: ''
  instance_name: ''
  attribute_name: 'material_at'
  values:
  - {key: 'm', value: '${mats[4]}'}
  - {key: 'wp', value: 'wp0'}
  is_negative: True";
done;

rosservice call /rosplan_knowledge_base/update_array "$type $param"
