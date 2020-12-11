(define (problem task)
(:domain turtlebot)
(:objects
    wp0 wp1 wp2 wp3 - waypoint
    robot01 robot02 - robot
    i01 i02 i03 - item
)
(:init
    (robot_at robot01 wp0)
    (robot_at robot02 wp0)

    (not_localised robot01)
    (not_localised robot02)

    (docked robot01)
    (docked robot02)

    (dock_at wp0)

    (material_at black wp0)
    (material_at blue wp1)
    (material_at green wp2)
    (material_at white wp3)

    (at 358.563 (material_at red wp0))
    (at 418.563 (not (material_at red wp0)))
    (at 958.563 (material_at red wp0))
    (at 1018.56 (not (material_at red wp0)))
    (at 1558.56 (material_at red wp0))
    (at 1618.56 (not (material_at red wp0)))
    (at 2158.56 (material_at red wp0))
    (at 2218.56 (not (material_at red wp0)))
    (at 2758.56 (material_at red wp0))
    (at 2818.56 (not (material_at red wp0)))
    (at 3358.56 (material_at red wp0))
    (at 3418.56 (not (material_at red wp0)))
    (at 3958.56 (material_at red wp0))
    (at 4018.56 (not (material_at red wp0)))
    (at 4558.56 (material_at red wp0))
    (at 4618.56 (not (material_at red wp0)))

    (not_busy robot01)
    (not_busy robot02)

    (= (carrying robot01 black) 0)
    (= (carrying robot01 blue) 0)
    (= (carrying robot01 green) 0)
    (= (carrying robot01 white) 0)

    (= (requires i01 black) 1)
    (= (requires i02 black) 1)
    (= (requires i03 black) 1)
    (= (requires i01 blue) 0)
    (= (requires i02 blue) 1)
    (= (requires i03 blue) 1)
    (= (requires i01 green) 0)
    (= (requires i02 green) 0)
    (= (requires i03 green) 0)
    (= (requires i01 red) 1)
    (= (requires i02 red) 0)
    (= (requires i03 red) 1)
    (= (requires i01 white) 1)
    (= (requires i02 white) 0)
    (= (requires i03 white) 1)


)
(:goal (and
    (complete i01 wp1)
    (complete i02 wp1)
    (complete i03 wp1)
))
)
