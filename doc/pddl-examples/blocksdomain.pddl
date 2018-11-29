(define (domain adl-blocksworld)
  (:requirements :adl :equality)
  (:types block)
  (:predicates (on ?x ?y) (clear ?x))

  (:action move
     :parameters (?b - block ?x ?y)
     :precondition (and
     		    (not(= ?b ?y))
				(clear ?b) (on ?b ?x) (clear ?y))
     :effect (and (on ?b ?y)
     	     	  (not (on ?b ?x))
		  (clear ?x)
		  (when (not (= ?y table))
		  	(not (clear ?y))))
  )
)