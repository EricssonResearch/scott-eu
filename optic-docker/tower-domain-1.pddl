(define (domain hanoi-domain)
	(:requirements :equality)
	(:predicates (disk ?x) (smaller ?x ?y) (on ?x ?y) (clear ?x))
	
	(:action move-disk
		:parameters (?disk ?below-disk ?new-below-disk)
		:precondition(and (disk ?disk)(smaller ?disk ?new-below-disk)
						(not (= ?new-below-disk ?below-disk))
						(not (= ?new-below-disk ?disk))
						(not (= ?below-disk ?disk))
						(on ?disk ?below-disk)
						(clear ?disk)
						(clear ?new-below-disk))
		:effect
			(and (clear ?below-disk)(on ?disk ?new-below-disk)
				(not (on ?disk ?below-disk))(not (clear ?new-below-disk)))
	)
)