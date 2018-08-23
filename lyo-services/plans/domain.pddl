(define (domain adl-blocksworld)
  (:requirements :typing :equality :fluents)
  (:types block location)
  (:constants table - location)
  (:predicates
    (on ?x ?y - (either location block))
    (clear ?x - (either location block))
  )
  (:functions
    (total-moved)
    (moved ?m - block)
  )
  (:action move
    :parameters
      (?b - block ?x ?y - (either location block))
    :precondition
      (and
        (not
          (= ?b ?y)
        )
        (clear ?b)
        (on ?b ?x)
        (clear ?y)
      )
    :effect
      (and
        (on ?b ?y)
        (not
          (on ?b ?x)
        )
        (clear ?x)
        (increase (moved ?b) 1)
        (increase (total-moved) 1)
        (when
          (not
            (= ?y table)
          )
          (not
            (clear ?y)
          )
        )
      )
  )
)
(define (problem adl-blocksworld-problem)
  (:domain adl-blocksworld)
  (:objects c - block table - location a b - block)
  (:init
    (= (moved a) 0)
    (= (moved c) 0)
    (= (moved b) 0)
    (= (total-moved) 0)
    (clear table)
    (clear b)
    (on b table)
    (on c a)
    (clear c)
    (on a table)
  )
  (:goal
    (or
      (on c b)
      (on b c)
    )
  )
  (:metric minimize (total-time))
)
