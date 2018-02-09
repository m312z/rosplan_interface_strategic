(define (domain turtlebot)

(:requirements :strips :typing :numeric-fluents :durative-actions :conditional-effects :equality)

(:types
	waypoint 
	robot
	item
	material
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

	(undocked ?r - robot)
	(docked ?r - robot)
	(dock_at ?wp - waypoint)

	(material_at ?m - material ?wp - waypoint)
	(complete ?i - item ?wp - waypoint)

	(strategic_planning)
)

(:functions
	(carrying ?r -robot ?m - material)
	(requires ?i - item ?m - material)
)

;; Move to any waypoint, avoiding terrain
(:durative-action goto_waypoint
	:parameters (?r - robot ?from ?to - waypoint)
	:duration ( = ?duration 60)
	:condition (and
		(at start (robot_at ?r ?from))
		(at start (localised ?r))
		(over all (undocked ?r))
		)
	:effect (and
		(at end (robot_at ?r ?to))
		(at start (not (robot_at ?r ?from)))
		)
)

;; Localise
(:durative-action localise
	:parameters (?r - robot)
	:duration ( = ?duration 60)
	:condition (and
		(over all (undocked ?r))
		)
	:effect (at end (localised ?r))
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

(:durative-action complete_mission
	:parameters (?r - robot ?i - item ?wp - waypoint)
	:duration ( = ?duration 3)
	:condition (and
		(over all (robot_at ?r ?wp))
		(at start (strategic_planning))
		)
	:effect (and
		(at end (complete ?i ?wp))
		)
)

)
