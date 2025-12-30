(define (domain assignment)
(:requirements :strips :typing :adl :fluents :durative-actions)

(:types
    robot
)

(:predicates
    (pipeline_ready ?r - robot)
    (pipeline_explored ?r - robot)
    (pipeline_captured ?r - robot)
)

(:functions
)

(:durative-action explore
    :parameters (?r - robot)
    :duration ( = ?duration 5)
    :condition (and
        (at start (pipeline_ready ?r))
    )
    :effect (and
        (at start (not (pipeline_ready ?r)))
        (at end (pipeline_explored ?r))
    )
)

(:durative-action capture
    :parameters (?r - robot)
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