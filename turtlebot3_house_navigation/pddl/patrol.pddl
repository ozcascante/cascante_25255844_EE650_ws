(define (domain patrol)
(:requirements :strips :typing :durative-actions)

(:types
  robot
  room
)

(:predicates
  (robot_at ?r - robot ?rm - room)
  (connected ?rm1 ?rm2 - room)
  (patrolled ?rm - room)
  (occupied ?rm - room)
  (light_on ?rm - room)
)

(:durative-action move
  :parameters (?r - robot ?rm1 ?rm2 - room)
  :duration (= ?duration 5)
  :condition (and
    (at start (connected ?rm1 ?rm2))
    (at start (robot_at ?r ?rm1))
  )
  :effect (and
    (at start (not (robot_at ?r ?rm1)))
    (at end (robot_at ?r ?rm2))
  )
)

(:durative-action patrol
  :parameters (?r - robot ?rm - room)
  :duration (= ?duration 5)
  :condition (and
    (at start (robot_at ?r ?rm))
  )
  :effect (and
    (at end (patrolled ?rm))
  )
)

)


