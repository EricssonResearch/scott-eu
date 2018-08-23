(define (problem scott-warehouse-problem)
  (:domain scott-warehouse)
  (:objects sh1 sh2 sh3 - shelf rb1 rb2 rb3 - robot cb1 cb2 cb3 - conveyor-belt b1 b2 b3 - box x0 y0 x1 y1 x2 y2 x10 y10 x20 y20 - coord)
  (:init
    (shelf-at sh1 x10 y0)
    (shelf-at sh2 x10 y10)
    (shelf-at sh3 x10 y20)

    (robot-at rb1 x0 y0)
    (robot-at rb2 x0 y1)
    (robot-at rb3 x0 y2)

    (free-robot rb1)
    (free-robot rb2)
    (free-robot rb3)

    (belt-at cb1 x20 y0)
    (belt-at cb2 x20 y10)
    (belt-at cb3 x20 y20)

    ;; dynamic part of the state, though the "static" one changes often, according to the experts

    (on-shelf b1 sh1)
    (on-shelf b2 sh2)
    (on-shelf b3 sh3)
  )
  (:goal
    (and
      (on-belt b1 cb1)
      (on-belt b2 cb2)
      (on-belt b3 cb3)
    )
  )
  (:metric minimize (total-time))
)
