; Warehouse domain at the waypoint level to match the generated file

(define (domain trs-safety-ws-domain)
	(:requirements
        ; :adl
        :typing
        ; :negative-preconditions
        ; :conditional-effects
        ; :fluents
    )

	(:types
		Box
        ConveyorBelt
        Shelf
        Robot
		Place ; TODO remove
		Waypoint
		Object ; TODO remove
		; Charge
	)

	; (:functions
	; 	;(capacity ?r - Robot)
	; )

	(:predicates
		(robot-at ?rb - Robot ?wp - Waypoint)
		(shelf-at ?sh - Shelf ?wp - Waypoint)
		(belt-at ?cb - ConveyorBelt ?wp - Waypoint)
		(on-belt ?b - Box ?cb - ConveyorBelt)
		(on-shelf ?b - Box ?sh - Shelf)
		(on-robot ?b - Box ?rb - Robot)
        (free-robot ?rb - Robot)

		(can-move ?w1 ?w2 - Waypoint)
	)

	(:action move-to-wp
		:parameters (?rb - Robot ?from ?to - Waypoint)
		:precondition (and
			(robot-at ?rb ?from)
    		(can-move ?from ?to)
		)
		:effect (and
			(robot-at ?rb ?to)
			(not (robot-at ?rb ?from))
		)
	)

	(:action pickupShelf
		:parameters (?robot - Robot ?box - Box ?shelf - Shelf ?waypoint - Waypoint)
		:precondition (and
			(robot-at ?robot ?waypoint)
			(shelf-at ?shelf ?waypoint)
			(on-shelf ?box ?shelf)
            (free-robot ?rb)
        )
		:effect (and
			(not (on-shelf ?box ?shelf))
            (not (free-robot ?rb))
			(on-robot ?box ?robot)
        )
    )

    (:action dropBelt
		:parameters (?robot - Robot ?box - Box ?belt - ConveyorBelt ?waypoint - Waypoint)
		:precondition (and
			(robot-at ?robot ?waypoint)
			(belt-at ?belt ?waypoint)
			(on-robot ?box ?robot))
		:effect (and
			(not (on-robot ?box ?robot))
			(on-belt ?box ?belt)))
)
