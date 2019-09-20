(define (domain scott-warehouse)
  (:requirements
    :typing
    :equality
    :fluents)
  (:types box - object robot conveyor-belt shelf - twin coord)
  (:constants table - location)
  (:predicates
    (robot-at ?rb - robot ?x ?y - coord)

    (on-belt  ?b - box ?cb - conveyor-belt)
    (on-shelf ?b - box ?sh - shelf)
    (on-robot ?b - box ?rb - robot)
    (free-robot ?rb)

    ; not sure how to declare some predicates as invariant
    (shelf-at ?o - shelf ?x ?y - coord)
    (belt-at  ?o - conveyor-belt ?x ?y - coord)
  )
  (:action move-to
    :parameters (?rb - robot ?x1 ?y1 - coord ?x2 ?y2 - coord)
    :precondition (and (robot-at ?rb ?x1 ?y1))
    :effect (and (robot-at ?rb ?x2 ?y2))
  )

  (:action pick-shelf
    :parameters (?rb - robot ?b - box ?sh - shelf ?x ?y - coord)
    :precondition (and (on-shelf ?b ?sh)
                      (shelf-at ?sh ?x ?y)
                      (robot-at ?rb ?x ?y)
                      (free-robot ?rb)
                  )
    :effect (and (on-robot ?b ?rb)
                (not (on-shelf ?b ?sh))
                (not (free-robot ?rb))
            )
  )

  (:action drop-belt
    :parameters (?rb - robot ?b - box ?cb - conveyor-belt ?x ?y - coord)
    :precondition (and (on-robot ?b ?rb)
                      (belt-at ?cb ?x ?y)
                      (robot-at ?rb ?x ?y)
                  )
    :effect (and (on-belt ?b ?cb)
                (not (on-robot ?b ?rb))
                (free-robot ?rb)
            )
  )
)
