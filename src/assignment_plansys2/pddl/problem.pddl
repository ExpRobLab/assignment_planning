(define (problem example_problem) (:domain example2)
(:objects
    robot1 - robot
    marker1 marker2 marker3 marker4 marker0 - marker
)
(:init
  (robot_at robot1 marker0)
  (unoccupied_robot robot1)
  (undetected marker1)
  (undetected marker2)
  (undetected marker3)
  (undetected marker4)
  (unphotographed marker1)
  (unphotographed marker2)
  (unphotographed marker3)
  (unphotographed marker4)
  
  (=(detected_marker)0)
  (=(current_id)1)
  (=(fake_id marker0)0)
  (=(fake_id marker1)1)
  (=(fake_id marker2)2)
  (=(fake_id marker3)3)
  (=(fake_id marker4)4)
)
(:goal
    (and
        (photographed marker1)
        (photographed marker2)
        (photographed marker3)
        (photographed marker4)
    )
)
)
