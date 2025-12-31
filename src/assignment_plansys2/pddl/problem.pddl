(define (problem assignment_problem)
(:domain assignment)
(:objects
    mogi_bot - robot
    wp1 wp2 wp3 wp4 - waypoint
)
(:init
    (pipeline_ready mogi_bot)
)
(:goal (and
    (pipeline_captured mogi_bot)
))
)
