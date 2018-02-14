#!/bin/bash

mats=(
	'black'
	'blue'
	'green'
	'red'
	'white'
   )

required_mats_i01=(1 0 0 1 1)
required_mats_i02=(1 1 0 0 0)
required_mats_i03=(1 1 0 1 1)

param="knowledge:
- knowledge_type: 0
  instance_type: 'item'
  instance_name: 'i01'
  attribute_name: ''
  function_value: 0.0
- knowledge_type: 0
  instance_type: 'item'
  instance_name: 'i02'
  attribute_name: ''
  function_value: 0.0
- knowledge_type: 0
  instance_type: 'item'
  instance_name: 'i03'
  attribute_name: ''
  function_value: 0.0";

for i in $(seq 0 4 )
do
param="$param
- knowledge_type: 2
  instance_type: ''
  instance_name: ''
  attribute_name: 'requires'
  values:
  - {key: 'i', value: 'i01'}
  - {key: 'm', value: '${mats[$i]}'}
  function_value: $((${required_mats_i01[$i]} + 0))
- knowledge_type: 2
  instance_type: ''
  instance_name: ''
  attribute_name: 'requires'
  values:
  - {key: 'i', value: 'i02'}
  - {key: 'm', value: '${mats[$i]}'}
  function_value: $((${required_mats_i02[$i]} + 0))
- knowledge_type: 2
  instance_type: ''
  instance_name: ''
  attribute_name: 'requires'
  values:
  - {key: 'i', value: 'i03'}
  - {key: 'm', value: '${mats[$i]}'}
  function_value: $((${required_mats_i03[$i]} + 0))";
done;

rosservice call /kcl_rosplan/update_knowledge_base_array "update_type: 0
$param"

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
