(define (domain scott-warehouse)
  (:requirements :typing :equality :fluents)
  (:types conveyor-belt box shelf robot coord)
  (:predicates
    (on-belt ?b - box ?cb - conveyor-belt)
    (robot-at ?rb - robot ?x ?y - coord)
    (shelf-at ?sh - shelf ?x ?y - coord)
    (on-robot ?b - box ?rb - robot)
    (belt-at ?cb - conveyor-belt ?x ?y - coord)
    (free-robot ?rb - robot)
    (on-shelf ?b - box ?sh - shelf)
  )
  (:action drop-belt
    :parameters
      (?rb - robot ?b - box ?cb - conveyor-belt ?x ?y - coord)
    :precondition
      (and
        (on-robot ?b ?rb)
        (belt-at ?cb ?x ?y)
        (robot-at ?rb ?x ?y)
      )
    :effect
      (and
        (on-belt ?b ?cb)
        (not
          (on-robot ?b ?rb)
        )
        (free-robot ?rb)
      )
  )
  (:action move-to
    :parameters
      (?rb - robot ?x1 ?y1 ?x2 ?y2 - coord)
    :precondition
      (and
        (robot-at ?rb ?x1 ?y1)
      )
    :effect
      (and
        (robot-at ?rb ?x2 ?y2)
      )
  )
  (:action pick-shelf
    :parameters
      (?rb - robot ?b - box ?sh - shelf ?x ?y - coord)
    :precondition
      (and
        (on-shelf ?b ?sh)
        (shelf-at ?sh ?x ?y)
        (robot-at ?rb ?x ?y)
        (free-robot ?rb)
      )
    :effect
      (and
        (on-robot ?b ?rb)
        (not
          (on-shelf ?b ?sh)
        )
        (not
          (free-robot ?rb)
        )
      )
  )
)
