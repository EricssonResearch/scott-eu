(define (problem adl-blocksworld-problem)
  (:domain adl-blocksworld)
  (:objects a b c - block table)

  (:init
     (on b table) (on a table) (on c a)
     (clear b) (clear c) (clear table)
  )

  (:goal
     (or (on b c) (on c b)))
	
	(:metric minimize (total-time))
)