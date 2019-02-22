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

	; (:constants

	; )

    ; TODO enable
	; (:functions
    ;     ; let's assume 1
	; 	;(capacity ?r - Robot)

    ;     ; (charge-level ?r - Robot)
	; 	; (max-Charge ?r - Robot)
	; 	; (low-Charge ?r - Robot)
	; 	; (high-Charge ?r - Robot)
	; )

	(:predicates
		(can-move ?w1 ?w2 - Waypoint)
		(robot-at ?rb - Robot ?wp - Waypoint)
		(shelf-at ?sh - Shelf ?wp - Waypoint)
		(belt-at ?cb - ConveyorBelt ?wp - Waypoint)
		(on-belt ?b - Box ?cb - ConveyorBelt)
		(on-shelf ?b - Box ?sh - Shelf)
		(on-robot ?b - Box ?rb - Robot)
        (free-robot ?rb - Robot)

        (situated-at ?p - Place ?w - Waypoint)
		(is-on ?o - Object ?p - Place)
		(carrying ?r - Robot ?o - Object)

        ; (is-recharging ?r - Robot)
		; (is-charging-station ?p - Place)
	)

	(:action move-to-wp
		:parameters (?rb - Robot ?from ?to - Waypoint)
		:precondition (and
			(robot-at ?rb ?from)
			(can-move ?from ?to)
			; (not (is-recharging ?robot))
			; (> (charge-level ?robot) (low-Charge ?robot))
		)
		:effect (and
			(robot-at ?rb ?to)
			(not (robot-at ?rb ?from))
		;	(decrease (charge-level ?robot) (required-charge (current-load ?robot) ?from-waypoint ?to-waypoint))
			; (decrease (charge-level ?robot) 2)
		)
	)

	(:action pickupShelf
		:parameters (?robot - Robot ?box - Box ?shelf - Shelf ?waypoint - Waypoint)
		:precondition (and
			(robot-at ?robot ?waypoint)
			(shelf-at ?shelf ?waypoint)
			(on-shelf ?box ?shelf))
		:effect (and
			(not (on-shelf ?box ?shelf))
			(on-robot ?box ?robot)))

    (:action dropBelt
		:parameters (?robot - Robot ?box - Box ?belt - ConveyorBelt ?waypoint - Waypoint)
		:precondition (and
			(robot-at ?robot ?waypoint)
			(belt-at ?belt ?waypoint)
			(on-robot ?box ?robot))
		:effect (and
			(not (on-robot ?box ?robot))
			(on-belt ?box ?belt)))

	; (:action pickupAtPlace
	; 	:parameters (?robot - Robot ?object - Object ?place - Place ?waypoint - Waypoint
	; 	)
	; 	:precondition (and
	; 		(robot-at ?robot ?waypoint)
	; 		(situated-at ?place ?waypoint)
	; 		(is-on ?object ?place)
	; 		; (not (is-recharging ?robot))
	; 		; (> (capacity ?robot) 0)
	; 	)
	; 	:effect (and
	; 		(not (is-on ?object ?place))
	; 		(carrying ?robot ?object)
	; 		; (decrease (capacity ?robot) 1)
	; 	;	(decrease (charge-level ?robot) (required-charge-pickup-drop (weight ?object)))
	; 		; (decrease (charge-level ?robot) 1)
	; 	)
	; )

	; (:action dropAtPlace
	; 	:parameters (?robot - Robot ?object - Object ?place - Place ?waypoint - Waypoint
	; 	)
	; 	:precondition (and
	; 		(robot-at ?robot ?waypoint)
	; 		(situated-at ?place ?waypoint)
	; 		(carrying ?robot ?object)
	; 		; (not (is-recharging ?robot))
	; 	)
	; 	:effect (and
	; 		(not (carrying ?robot ?object))
	; 		(is-on ?object ?place)
	; 		; (increase (capacity ?robot) 1)
	; 	;	(decrease (charge-level ?robot) (required-charge-pickup-drop (weight ?object)))
	; 		; (decrease (charge-level ?robot) 1)
	; 	)
	; )



	; ; recharge can be done at any time, the effect is that it should be more than the high-charge
	; ; level of the hysteresis loop
	; (:action startRecharge
	; 	:parameters (?robot - Robot ?RechargeStationName - Place ?waypoint - Waypoint)
	; 	:precondition (and
	; 		(robot-at ?robot ?waypoint)
	; 		(situated-at ?RechargeStationName ?waypoint)
	; 		(is-charging-station ?RechargeStationName)
	; 		; (< (charge-level ?robot) (max-charge ?robot))
	; 	)
	; 	:effect (and
	; 		; (is-recharging ?robot)
	; 		; (increase (charge-level ?robot) 1)
	; 	)
	; )

	; (:action stopRecharge
	; 	:parameters (?robot - Robot ?RechargeStationName - Place ?waypoint - Waypoint)
	; 	:precondition (and
	; 		(robot-at ?robot ?waypoint)
	; 		(situated-at ?RechargeStationName ?waypoint)
	; 		; (is-recharging ?robot)
	; 		; (>(charge-level ?robot) (high-Charge ?robot))
	; 	)
	; 	:effect (and
	; 		; (not(is-recharging ?robot))
	; 	)
	; )
)
