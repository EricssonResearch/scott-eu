(define (problem tower-of-hanoi)
	(:domain hanoi-domain)
	(:objects disk1 disk2 disk3 disk4 disk5)
	(:init (disk disk1)(disk disk2)(disk disk3)
			(smaller disk1 disk2)
			(smaller disk1 disk3)
			(smaller disk2 disk3)
			(smaller disk1 disk4)
			(smaller disk2 disk4)
			(smaller disk1 disk5)
			(smaller disk2 disk5)
			(on disk1 disk2)
			(on disk2 disk3)
			(clear disk1)
			(clear disk4)
			(clear disk5)
	)
	
	(:goal (and
			(on disk1 disk2)
			(on disk2 disk4))
	)

	(:metric minimize (total-time))
)

