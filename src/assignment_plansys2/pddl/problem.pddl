(define (problem assignment_problem)
(:domain assignment)
(:objects
    mogi_bot - robot
)
(:init
    (pipeline_ready mogi_bot)
)
(:goal (and
    (pipeline_captured mogi_bot)
))
)