(define (domain assignment)
(:requirements :strips :typing :adl :fluents :durative-actions)

(:types
    robot
    waypoint
)

(:predicates
  (mode_explore)
  (mode_capture)
  (explored ?wp - waypoint)
  (captured ?wp - waypoint)
  (executing ?r - robot)
)

(:durative-action navigate_and_rotate
  :parameters (?r - robot ?wp - waypoint)
  :duration (= ?duration 5)
  :condition (and
    (at start (mode_explore))
    (at start (not (executing ?r)))
    (at start (not (explored ?wp)))
  )
  :effect (and
    (at start (executing ?r))
    (at end (not (executing ?r)))
    (at end (explored ?wp))
  )
)

(:durative-action navigate_and_capture
  :parameters (?r - robot ?wp - waypoint)
  :duration (= ?duration 5)
  :condition (and
    (at start (mode_capture))
    (at start (not (executing ?r)))
    (at start (explored ?wp))
    (at start (not (captured ?wp)))
  )
  :effect (and
    (at start (executing ?r))
    (at end (not (executing ?r)))
    (at end (captured ?wp))
  )
)

(:action switch_to_capture
  :parameters ()
  :precondition (and
    (mode_explore)
    (explored w1) (explored w2) (explored w3) (explored w4)
  )
  :effect (and
    (not (mode_explore))
    (mode_capture)
  )
)
