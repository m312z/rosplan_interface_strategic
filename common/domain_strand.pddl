(define (domain turtlebot)

	(:requirements :strips :typing :numeric-fluents :durative-actions :conditional-effects :equality :timed-initial-literals)

	(:types
		waypoint 
		robot
		item
		material
		mission
			- object
	)

	(:constants
		black
		blue
		green
		red
		white
			- material	
	)

	(:predicates
		(robot_at ?r - robot ?wp - waypoint)
		(localised ?r - robot)
		(not_localised ?r - robot)
		(can_tac_goto)
		(can_strat_goto)

		(undocked ?r - robot)
		(docked ?r - robot)
		(dock_at ?wp - waypoint)

		(material_at ?m - material ?wp - waypoint)
		(complete ?i - item ?wp - waypoint)

		(mission_complete ?m - mission)
		(not_busy)
		(mission_at ?m - mission ?wp - waypoint)
		(mission_active ?m - mission)
		(end_point ?m - mission ?wp - waypoint)
	)

	(:functions
		(carrying ?r -robot ?m - material)
		(requires ?i - item ?m - material)
		(mission_duration ?m - mission)
	)

	;; Move to any waypoint, avoiding terrain - need to find a way to make this not execute at same time as complete_mission
	;; only way I can think right now is have to actions - one that can be performed in complete_mission and another that
	;; can be performed out of it - as if this can't execute at the same time as complete_mission then it can't be performed
	;; in it's tac plans
	(:durative-action tactical_goto
		:parameters (?r - robot ?from ?to - waypoint)
		:duration ( = ?duration 60)
		:condition (and
			(at start (robot_at ?r ?from))
			(at start (localised ?r))
			(over all (undocked ?r))
			(over all (can_tac_goto))
			)
		:effect (and
			(at start (not (robot_at ?r ?from)))
			(at end (robot_at ?r ?to))
			)
	)

	;; THIS ALSO DOESN'T WORK BECAUSE CAN STILL USE GOTO_WAYPOINT AT SAME TIME OF COMPLETE_MISSION - NEED TO BE USING IT IN 
	;; TACTIAL BUT NOT AT SAME TIME IN STRATEGIC
	(:durative-action strategic_goto
		:parameters (?r - robot ?from ?to - waypoint)
		:duration ( = ?duration 60)
		:condition (and
			(over all (not_busy))
			(at start (robot_at ?r ?from))
			(at start (localised ?r))
			(over all (undocked ?r))
			(over all (can_strat_goto))
			)
		:effect (and
			(at start (not (robot_at ?r ?from)))
			(at end (robot_at ?r ?to))
			)
	)

	;; Localise
	(:durative-action localise
		:parameters (?r - robot)
		:duration ( = ?duration 60)
		:condition (and
			(at start (not_localised ?r))
			(over all (undocked ?r))
			)
		:effect (and
			(at end (localised ?r))
			(at end (not (not_localised ?r)))
			)
	)

	;; Dock to charge
	(:durative-action dock
		:parameters (?r - robot ?wp - waypoint)
		:duration ( = ?duration 20)
		:condition (and
			(over all (dock_at ?wp))
			(at start (robot_at ?r ?wp))
			(at start (undocked ?r))
			)
		:effect (and
			(at end (docked ?r))
			(at start (not (undocked ?r)))
			)
	)

	(:durative-action undock
		:parameters (?r - robot ?wp - waypoint)
		:duration ( = ?duration 5)
		:condition (and
			(over all (dock_at ?wp))
			(at start (docked ?r))
			)
		:effect (and
			(at start (not (docked ?r)))
			(at end (undocked ?r))
			)
	)

	(:durative-action collect_black
		:parameters (?r - robot ?wp - waypoint)
		:duration ( = ?duration 10)
		:condition (and
			(over all (robot_at ?r ?wp))
			(over all (material_at black ?wp))
			)
		:effect (and
			(at end (assign (carrying ?r black) 1))
			)
	)

	(:durative-action collect_blue
		:parameters (?r - robot ?wp - waypoint)
		:duration ( = ?duration 120)
		:condition (and
			(over all (robot_at ?r ?wp))
			(over all (material_at blue ?wp))
			)
		:effect (and
			(at end (increase (carrying ?r blue) 1))
			)
	)

	(:durative-action collect_green
		:parameters (?r1 ?r2 - robot ?wp - waypoint)
		:duration ( = ?duration 20)
		:condition (and
			(over all (robot_at ?r1 ?wp))
			(over all (robot_at ?r2 ?wp))
			(over all (not (= ?r1 ?r2)))
			(over all (material_at green ?wp))
			)
		:effect (and
			(at end (increase (carrying ?r1 green) 1))
			)
	)

	(:durative-action collect_red
		:parameters (?r - robot ?wp - waypoint)
		:duration ( = ?duration 10)
		:condition (and
			(over all (robot_at ?r ?wp))
			(over all (material_at red ?wp))
			)
		:effect (and
			(at end (increase (carrying ?r red) 1))
			)
	)

	(:durative-action collect_white
		:parameters (?r - robot ?wp - waypoint)
		:duration ( = ?duration 10)
		:condition (and
			(over all (robot_at ?r ?wp))
			(over all (material_at white ?wp))
			)
		:effect (and
			(at end (increase (carrying ?r white) 1))
			)
	)

	(:durative-action complete_building
		:parameters (?r - robot ?i - item ?wp - waypoint)
		:duration ( = ?duration 3)
		:condition (and
			(over all (robot_at ?r ?wp))
			(at start (forall (?m - material) (<= (requires ?i ?m) (carrying ?r ?m))))
			)
		:effect (and
			(at end (complete ?i ?wp))
			(at end (decrease (carrying ?r black) (requires ?i black)))
			(at end (decrease (carrying ?r blue) (requires ?i blue)))
			(at end (decrease (carrying ?r green) (requires ?i green)))
			(at end (decrease (carrying ?r red) (requires ?i red)))
			(at end (decrease (carrying ?r white) (requires ?i white)))
			)
	)

	;;need to add (at end (robot_at ?r ?to)) effect and (at start (mission_active ?m)) precondition
	(:durative-action complete_mission
		:parameters (?r - robot ?m - mission ?from ?to - waypoint)
		:duration ( = ?duration (mission_duration ?m))
		:condition (and
			(at start (undocked ?r))
			(at start (localised ?r))
			(at start (not_busy))
			(at start (robot_at ?r ?from))
			(over all (mission_at ?m ?from))
			(at end (end_point ?m ?to))
			)
		:effect (and
			(at start (not (not_busy)))
			(at end (not (robot_at ?r ?from)))
			(at end (robot_at ?r ?to))
			(at end (not (mission_at ?m ?from)))
			(at end (mission_complete ?m))
			(at end (not_busy))
			(at end (not (end_point ?m ?to)))
			)
	)
)