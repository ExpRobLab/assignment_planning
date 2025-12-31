(define (domain assignment)
(:requirements :strips :typing :adl :fluents :durative-actions)

(:types
    robot
    waypoint
)

(:predicates
    (pipeline_ready ?r - robot)
    (pipeline_explored ?r - robot)
    (pipeline_captured ?r - robot)
)

(:durative-action navigate_and_rotate
    :parameters (?r - robot ?wp - waypoint)
    :duration ( = ?duration 5)
    :condition (and
        (at start (pipeline_ready ?r))
    )
    :effect (and
        (at start (not (pipeline_ready ?r)))
        (at end (pipeline_explored ?r))
    )
)

(:durative-action navigate_and_capture
    :parameters (?r - robot ?wp - waypoint)
    :duration ( = ?duration 5)
    :condition (and
        (at start (pipeline_explored ?r))
    )
    :effect (and
        (at start (not (pipeline_explored ?r)))
        (at end (pipeline_captured ?r))
    )
)
)
