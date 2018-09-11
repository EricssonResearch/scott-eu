(define (domain scott-mir)
  (:requirements
    :typing
    :equality
    :fluents)
  (:types
    Robot
    ; "A mission is a predefined series of actions that the robot can be set to perform by the click of a button."
    Mission
    Marker
    Action
    Site
    PlcRegister
    ; TODO would be good to use real ints for this
    order-no
    action-transport action-adjust-localisation action-dock action-move action-move-coord action-move-relative action-set-footprint action-switch-map - action
    )
  (:constants
    eab-tallin1 - Site
    charger1 charger2 - Marker)
  (:predicates
    ;"The name must be unique and is used to identify the mission."
    (mission-name ?m - Mission)
    ; §4.2.2 has a "site" field in the figure
    (mission-site ?m - Mission ?s - Site)
    ; §4.2.2 "A mission is made up of actions"
    (action-mission ?a - Action ?m - Mission ?no - order-no)
    ; "You can also pick already created missions and embed them in new missions."
    (sub-mission ?sm - Mission ?m - Mission ?no - order-no)

    (robot-adjusted ?rb - Robot)

    (plc-value ?r - PlcRegister)

    ; TODO add some fluents to cover the logic on p. 36
  )
  (:action move-adjust-localisation
    :parameters (?rb - Robot)
    :precondition (not (robot-docked ?rb))
    :effect (robot-adjusted ?rb)
  )

  (:action move-dock
    :parameters (?rb - Robot ?m - marker)
    :precondition (robot-adjusted ?rb)
    :effect (robot-docked ?rb) ; universal quantification restriction in PDDL prevents using (robot-docked ?rb ?m)
  )

  (:action robot-charge
  ; TODO charge condition combos
    :parameters (?rb - Robot ?m - marker)
    :precondition (and (robot-docked ?rb ?m)
                    ; TODO charge is <= 40%, for example
                    )
    :effect (
        ; TODO
    )
  )

  ; TODO moves w/orientation and w/o safety p. 32
  ; TODO switch map
  ; TODO break, continue, loop, pause, return, wait, while
  ; TODO if-{bluetooth,plc,missions,battery} (eq, neq, lt, lte, gt, gte)
  ; TODO prompts
  ; TODO log, throw, try/catch
  ; TODO play sound, show light
  ; TODO set-register, set-io
  ; TODO wait-register, wait-io
  ; TODO send-email

  ; TODO **VARIABLES** would translate into plans with universal quantifiers!

  ; TODO
  ; TODO
  ; TODO
  ; TODO
  ; TODO
  ; TODO
  ; TODO
  ; TODO
  ; TODO
  ; TODO


  ; (:action move-to
  ;   :parameters (?rb - robot ?x1 ?y1 - coord ?x2 ?y2 - coord)
  ;   :precondition (and (robot-at ?rb ?x1 ?y1))
  ;   :effect (and (robot-at ?rb ?x2 ?y2))
  ; )

  ; (:action pick-shelf
  ;   :parameters (?rb - robot ?b - box ?sh - shelf ?x ?y - coord)
  ;   :precondition (and (on-shelf ?b ?sh)
  ;                     (shelf-at ?sh ?x ?y)
  ;                     (robot-at ?rb ?x ?y)
  ;                     (free-robot ?rb)
  ;                 )
  ;   :effect (and (on-robot ?b ?rb)
  ;               (not (on-shelf ?b ?sh))
  ;               (not (free-robot ?rb))
  ;           )
  ; )

  ; (:action drop-belt
  ;   :parameters (?rb - robot ?b - box ?cb - conveyor-belt ?x ?y - coord)
  ;   :precondition (and (on-robot ?b ?rb)
  ;                     (belt-at ?cb ?x ?y)
  ;                     (robot-at ?rb ?x ?y)
  ;                 )
  ;   :effect (and (on-belt ?b ?cb)
  ;               (not (on-robot ?b ?rb))
  ;               (free-robot ?rb)
  ;           )
  ; )
)
