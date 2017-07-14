(define (problem adl-blocksworld-problem)
  (:domain adl-blocksworld)
  (:objects a b c - block table)

  (:init
     (on b table) (on a table) (on c table)
     (clear b) (clear c)(clear a)(clear table)
  )

  (:goal
     (forall (?x - block) (imply (clear ?x)(exists(?y - block)(on ?x ?y))))
	 ;(or (clear b)(on b table))
   )
	
	(:metric minimize (total-time))
)