(define (stream nightout)

  (:stream s-cash
    :outputs (?c)
    :certified (cash ?c))

  (:stream t-geq
    :inputs (?c1 ?c2)
    :domain (and (cash ?c1) (cash ?c2))
    :certified (geq ?c1 ?c2))

  ;(:function (Duration ?t)
  ;           (Traj ?t))
)