(define (problem assignment_problem)
(:domain assignment)
(:objects
    mogi_bot - robot
    wp1 wp2 wp3 wp4 - waypoint
)
(:init
  (mode_explore)
)
(:goal (and
  (captured w1)
  (captured w2)
  (captured w3)
  (captured w4)
))
)
