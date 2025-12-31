(define (domain navigation)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
marker
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_at ?r - robot ?ro - marker)
(detected ?ro - marker)
(undetected ?ro - marker)
(photographed ?ro - marker)
(unphotographed ?ro - marker)
(unoccupied_robot ?r - robot)
);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions
    (fake_id ?m - marker)
    (detected_marker)
    (current_id)

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;



;;The robot searchs for the marker in the evironment
(:durative-action move_to_detect
    :parameters (?r - robot ?to - marker ?from - marker)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?from))
        (at start(undetected ?to))
        (at start (unoccupied_robot ?r))
        (at start (<(detected_marker)4))
        )
    :effect (and
        (at start(not(robot_at ?r ?from)))
        (at start(not(unoccupied_robot ?r)))
        (at end(robot_at ?r ?to))
        (at end(detected ?to))
        (at end(not(undetected ?to)))
        (at end(unoccupied_robot ?r))
        (at end(increase(detected_marker)1))
    )
)

;;Change the state from searching to scanning and the robot goes back to the initial point
(:durative-action change_state
    :parameters (?r - robot ?to - marker ?from - marker)
    :duration ( = ?duration 5)
    :condition (and
        (at start (robot_at ?r ?from))
        (at start(=(detected_marker)4))
        )
    :effect (and
        (at start(not(robot_at ?r ?from)))
        (at end (robot_at ?r ?to))
        (at end(increase(detected_marker)1))
    )
)


;;The robot goes to the marker in order
(:durative-action move_to_photograph
    :parameters (?r - robot ?to - marker ?from - marker)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?from))
        (at start(=(fake_id ?to)(current_id)))
        (at start(unphotographed ?to))
        (at start(detected ?to))
        (at start (unoccupied_robot ?r))
        (at start (>(detected_marker)4))
        )
    :effect (and
        (at start(not(robot_at ?r ?from)))
        (at start(not(unoccupied_robot ?r)))
        (at end(robot_at ?r ?to))
        (at end(photographed ?to))
        (at end(not(unphotographed ?to)))
        (at end(unoccupied_robot ?r))
        (at end(increase(current_id)1))
    )
)



);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
