# rosplan_interface_strategic

This repository holds a package for strategic-tactical planning with rosplan. This includes:
- The tactical action interface, which performs planning and execution as an action within a strategic plan.
- An example launch file and scripts.
- An example domain file.

This README describes how to run a version of strategic-tactical planning in ROSPlan. It is split into the following parts:

1) Describing the PDDL domain and problem used in the example, and how it can be used for strategic-tactical planning.
2) Detailing the ROS nodes used in the example launch.
3) Instructions for running the example.

## The PDDL domain and problem

The PDDL domain file is contained in the folder "common".

The domain describes a robot collecting materials to build an item at a waypoint. Given sufficient items the problem becomes too difficult to solve. Thus, some goals must be abstracted and replaced "missions". The flow is as follows:

1. The goals are decomposed into sub-tasks (missions).
2. A plan is generated for each mission, to associate the mission with an approximate duration.
3. The goals of the original problem become the completion of each mission.
4. A strategic plan is generated to complete all the missions, and the execution begins.

When the "complete_mission" action is executed,
1. The "complete_mission" goals are remoevd from the knowledge base.
2. The appropriate mission goal(s) are added to the knowledge base.
3. The tactical planning system is called, including dispatch.
4. The goal(s) are removed, and the previous goals are added again.
5. The plan success is sent back as action feedack to the strategic level.

A mission consists of the following PDDL constructs:
```
(:predicates
	(mission_complete ?m - mission)
	(not_busy)
```
```
(:functions
	(mission_duration ?m - mission)
```
```
(:durative-action complete_mission
	:parameters (?r - robot ?m - mission ?wp - waypoint)
	:duration ( = ?duration (mission_duration ?m))
	:condition (and
		(at start (not_busy))
		(over all (robot_at ?r ?wp))
		)
	:effect (and
		(at start (not (not_busy)))
		(at end (mission_complete ?m))
		(at end (not_busy))
		)
)
```
The robot is able to complete only one mission at a time, enforced by the "not_busy" predicate.

## The nodes in the launch file

- *strategic plan control* This node performs the problem decomposition as a service (steps 1-3 of the flow).
- *tactical plan control* This node is an action interface that listens for the "complete_mission" action (step 5 of the flow).

- *strategic/tactical planner interface*, *strategic/tactical problem generation*, *strategic/tactical plan parsing*, *strategic/tactical plan dispatch* These nodes produce a PDDL problem, pass it to a planner, parse the planner output, and dispatch the resulting plan, all as services.

## Running the Example

To run the example, open two terminals and source the workspace in both.

In the first terminal terminal we launch everything using the command:
```
roslaunch rosplan_interface_strategic strategic_tactical.launch
```

In the second terminal, run the three scripts.
```
rosrun rosplan_interface_strategic strategic_init.bash;
rosrun rosplan_interface_strategic strategic_goal.bash;
rosrun rosplan_interface_strategic strategic_plan.bash;
```
These scripts:
1. Add the initial state to the Knowledge Base (init)
2. Add the goal to the Knowledge Base (goal)
3. Call all the services mentioned above to complete the scenario (plan)

Optionally (but recommended) is to view the plans currenty under execution. To do this source the workspace and run:
```
rqt --standalone rosplan_rqt.esterel_plan_viewer & rqt --standalone rosplan_rqt.esterel_plan_viewer
```

In the first window view the strategic plan by entering the topic "/strategic\_plan\_dispatch/plan_graph" and pressing refresh.
In the second window view the tactical plan: "/tactical\_plan\_dispatch/plan_graph".

The windows should look like:
![plans](https://github.com/m312z/rosplan_interface_strategic/blob/master/readme_image.png)

