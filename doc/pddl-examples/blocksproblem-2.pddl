(define (problem adl-blocksworld-problem)
  (:domain adl-blocksworld)
  (:objects a b c - block table)

  (:init
     (on b table) (on a table) (on c a)
     (clear b) (clear c) (clear table)
  )

  (:goal
     (and (on a b) (on b c) (on c table)))
	
	(:metric minimize (total-time))
)