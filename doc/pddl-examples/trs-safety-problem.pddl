(define (problem scott-warehouse-problem)
  (:domain scott-warehouse)
  (:objects y8 y10 y19 y7 - coord rb1 - robot x3 x2 y15 y5 x23 - coord sh1 - shelf y13 y12 y20 - coord ob2 - robot x19 - coord ob1 - robot x10 x7 x6 x22 y0 y9 - coord rb3 rb2 - robot y17 y22 x4 y16 x12 x25 x24 y14 y2 y21 - coord ob3 - robot cb1 - conveyor-belt x11 x8 y1 x16 - coord rb4 - robot y18 y24 y23 x14 x13 y6 - coord b1 - box x1 y4 y3 x0 y11 x18 x17 x9 y25 x5 x21 x15 x20 - coord)
;   (:objects rb1 - robot b1 - box cb1 - conveyor-belt sh1 - shelf x0 x1 y1 y10 y20 - coord)
  (:init
    (on-shelf b1 sh1)
    (shelf-at sh1 x0 y10)
    (robot-at rb1 x1 y1)
    (free-robot rb1)
    ; (robot-at rb2 x1 y2)
    ; (free-robot rb2)
    ; (robot-at rb3 x1 y3)
    ; (free-robot rb3)
    ; (robot-at rb4 x1 y4)
    ; (free-robot rb4)
    (belt-at cb1 x0 y20)
    ; (robot-at ob2 x3 y3)  
    ; (robot-at ob3 x3 y4)
    ; (robot-at ob1 x3 y2)
  )
  (:goal
    (and
      (on-belt b1 cb1)
      (not (on-shelf b1 sh1))
    )
  )
;   (:metric minimize (total-time))
)
