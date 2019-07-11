; Warehouse domain at the waypoint level to match the generated file

(define (domain trs-safety-ws-domain)
	(:requirements :adl :typing :negative-preconditions :conditional-effects :fluents)

	(:types
		Robot
		Place
		Waypoint
		Object
		; Charge
	)

	(:constants

	)

	(:functions
        ; let's assume 1
		(capacity ?r - Robot)
		; (charge-level ?r - Robot)
		; (max-Charge ?r - Robot)
		; (low-Charge ?r - Robot)
		; (high-Charge ?r - Robot)
	)

	(:predicates
		(is-at ?r - Robot ?w - Waypoint)
		(situated-at ?p - Place ?w - Waypoint)
		(is-on ?o - Object ?p - Place)
		(carrying ?r - Robot ?o - Object)
		(can-move ?w1 ?w2 - Waypoint)
		; (is-recharging ?r - Robot)
		; (is-charging-station ?p - Place)
	)

	(:action moveToWaypoint
		:parameters (?robot - Robot ?from-waypoint ?to-waypoint - Waypoint)
		:precondition (and
			(is-at ?robot ?from-waypoint)
			(can-move ?from-waypoint ?to-waypoint)
			; (not (is-recharging ?robot))
			; (> (charge-level ?robot) (low-Charge ?robot))
		)
		:effect (and
			(is-at ?robot ?to-waypoint)
			(not (is-at ?robot ?from-waypoint))
		;	(decrease (charge-level ?robot) (required-charge (current-load ?robot) ?from-waypoint ?to-waypoint))
			; (decrease (charge-level ?robot) 2)
		)
	)

	(:action pickupAtPlace
		:parameters (?robot - Robot ?object - Object ?place - Place ?waypoint - Waypoint
		)
		:precondition (and
			(is-at ?robot ?waypoint)
			(situated-at ?place ?waypoint)
			(is-on ?object ?place)
			; (not (is-recharging ?robot))
			; (> (capacity ?robot) 0)
		)
		:effect (and
			(not (is-on ?object ?place))
			(carrying ?robot ?object)
			; (decrease (capacity ?robot) 1)
		;	(decrease (charge-level ?robot) (required-charge-pickup-drop (weight ?object)))
			; (decrease (charge-level ?robot) 1)
		)
	)

	(:action dropAtPlace
		:parameters (?robot - Robot ?object - Object ?place - Place ?waypoint - Waypoint
		)
		:precondition (and
			(is-at ?robot ?waypoint)
			(situated-at ?place ?waypoint)
			(carrying ?robot ?object)
			; (not (is-recharging ?robot))
		)
		:effect (and
			(not (carrying ?robot ?object))
			(is-on ?object ?place)
			(increase (capacity ?robot) 1)
		;	(decrease (charge-level ?robot) (required-charge-pickup-drop (weight ?object)))
			; (decrease (charge-level ?robot) 1)
		)
	)

	; ; recharge can be done at any time, the effect is that it should be more than the high-charge
	; ; level of the hysteresis loop
	; (:action startRecharge
	; 	:parameters (?robot - Robot ?RechargeStationName - Place ?waypoint - Waypoint)
	; 	:precondition (and
	; 		(is-at ?robot ?waypoint)
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
	; 		(is-at ?robot ?waypoint)
	; 		(situated-at ?RechargeStationName ?waypoint)
	; 		; (is-recharging ?robot)
	; 		; (>(charge-level ?robot) (high-Charge ?robot))
	; 	)
	; 	:effect (and
	; 		; (not(is-recharging ?robot))
	; 	)
	; )
)
