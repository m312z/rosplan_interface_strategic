from random import *
import os
import stat

mats = ['black', 'blue', 'green', 'red', 'white']
locs = {'black': [], 'blue': [], 'green': [], 'red': [], 'white': []}
def create_init_file(name, noOfWaypoints, noOfMats):
	with open(name + "strategic_init.bash", 'w') as file:
		file.write("#!/bin/bash\n\n")
		file.write("param=\"knowledge:\n- knowledge_type: 0\n  instance_type: 'robot'\n  instance_name: 'kenny'\n  attribute_name: ''\n  function_value: 0.0")

		#file.write("\nparam=\"$param")
		for i in range(noOfWaypoints):
			file.write("\n- knowledge_type: 0\n  instance_type: 'waypoint'\n  instance_name: 'wp" + str(i) + "'\n  attribute_name: ''\n  function_value: 0.0")
		for m in mats:	
			file.write("\n- knowledge_type: 2\n  instance_type: ''\n  instance_name: ''\n  attribute_name: 'carrying'\n  values:\n  - {key: 'r', value: 'kenny'}\n  - {key: 'm', value: '" + m + "'}\n  function_value: 0.0")
		
		#this method doesn't guarantee a colour of each type
		for i in  range(noOfMats):
			m = randint(0, 4)
			wp = randint(0, noOfWaypoints - 1)
			file.write("\n- knowledge_type: 1\n  instance_type: ''\n  instance_name: ''\n  attribute_name: 'material_at'\n  values:\n  - {key: 'm', value: '"+ mats[m] + "'}\n  - {key: 'wp', value: 'wp" + str(wp) + "'}\n  function_value: 0.0")
			locs[mats[m]].append("wp" + str(i))

		file.write("\n- knowledge_type: 1\n  instance_type: ''\n  instance_name: ''\n  attribute_name: 'not_busy'\n  function_value: 0.0")
		wp = randint(0, noOfWaypoints - 1)
		file.write("\n- knowledge_type: 1\n  instance_type: ''\n  instance_name: ''\n  attribute_name: 'robot_at'\n  values:\n  - {key: 'r', value: 'kenny'}\n  - {key: 'wp', value: 'wp" + str(wp) + "'}\n  function_value: 0.0")
		file.write("\n- knowledge_type: 1\n  instance_type: ''\n  instance_name: ''\n  attribute_name: 'docked'\n  values:\n  - {key: 'r', value: 'kenny'}\n  function_value: 0.0")
		dock = randint(0, noOfWaypoints - 1)
		file.write("\n- knowledge_type: 1\n  instance_type: ''\n  instance_name: ''\n  attribute_name: 'dock_at'\n  values:\n  - {key: 'wp', value: 'wp" + str(dock) + "'}\n  function_value: 0.0")
		file.write("\n- knowledge_type: 1\n  instance_type: ''\n  instance_name: ''\n  attribute_name: 'not_localised'\n  values:\n  - {key: 'r', value: 'kenny'}\n  function_value: 0.0")
		file.write("\";\n\n")
		file.write("rosservice call /kcl_rosplan/update_knowledge_base_array \"update_type: 0\n$param\"")

def create_goal_file(name, noOfItems, noOfWaypoints):
	with open(name + "strategic_goal.bash", 'w') as file:
		file.write("#!/bin/bash\n\n")
		file.write("param=\"knowledge:")
		for i in range(noOfItems):
			file.write("\n- knowledge_type: 0\n  instance_type: 'item'\n  instance_name: 'i" + str(i) + "'\n  attribute_name: ''\n  function_value: 0.0")
		file.write("\"")
		file.write("\nparam=\"$param")
		#this method doesn't guarantee all items will need at least one mat
		for i in range(noOfItems):
			for m in mats:
				if locs[m] != [] and m != 'green':
					needs = randint(0, 100)
					if(needs > 30):
						file.write("\n- knowledge_type: 2\n  instance_type: ''\n  instance_name: ''\n  attribute_name: 'requires'\n  values:\n  - {key: 'i', value: 'i" + str(i) + "'}\n  - {key: 'm', value: '" + m + "'}\n  function_value: 1.0")
					else:
						file.write("\n- knowledge_type: 2\n  instance_type: ''\n  instance_name: ''\n  attribute_name: 'requires'\n  values:\n  - {key: 'i', value: 'i" + str(i) + "'}\n  - {key: 'm', value: '" + m + "'}\n  function_value: 0.0")
				else:
					file.write("\n- knowledge_type: 2\n  instance_type: ''\n  instance_name: ''\n  attribute_name: 'requires'\n  values:\n  - {key: 'i', value: 'i" + str(i) + "'}\n  - {key: 'm', value: '" + m + "'}\n  function_value: 0.0")

		file.write("\";\n")
		file.write("\nrosservice call /kcl_rosplan/update_knowledge_base_array \"update_type: 0\n$param\"")
		file.write("\nparam2=\"knowledge:")
		for i in range(noOfItems):
			wp = randint(0, noOfWaypoints - 1)
			file.write("\n- knowledge_type: 1\n  instance_type: ''\n  instance_name: ''\n  attribute_name: 'complete'\n  values:\n  - {key: 'i', value: 'i" + str(i) + "'}\n  - {key: 'wp', value: 'wp" + str(wp) + "'}\n  function_value: 0.0")
		file.write("\";\n")
		file.write("rosservice call /kcl_rosplan/update_knowledge_base_array \"update_type: 1\n$param2\"")

def make_executable(path):
    mode = os.stat(path).st_mode
    mode |= (mode & 0o444) >> 2    # copy R bits to X
    os.chmod(path, mode)

def create_files(num):
	for i in range(num):
		waypoints = randint(3, 150)
		mats = randint(3, 150)
		items = randint(3, 30)
		name = "ex" + str(i)
		create_init_file(name, waypoints, mats)
		create_goal_file(name, items, waypoints)
		make_executable(name + "_strategic_init.bash")
		make_executable(name + "_strategic_goal.bash")

#name = raw_input("Enter name of file: ")
# for i in range(0, 1000):
# 	create_init_file("ex" + str(i), i, i)

create_init_file("", 100, 70)
create_goal_file("", 20, 100)
make_executable("strategic_init.bash")
make_executable("strategic_goal.bash")

#create_files(100)
