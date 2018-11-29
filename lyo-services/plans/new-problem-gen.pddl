(define (problem scott-warehouse-problem)
  (:domain scott-warehouse)
  (:objects b1 - box y0 - coord cb1 - conveyor-belt y10 - coord rb1 - robot x0 y20 - coord sh1 - shelf)
  (:init
    (robot-at rb1 x0 y0)
    (shelf-at sh1 x0 y10)
    (belt-at cb1 x0 y20)
    (on-shelf b1 sh1)
    (free-robot rb1)
  )
  (:goal
    (and
      (on-belt b1 cb1)
    )
  )
  (:metric minimize (total-time))
)
