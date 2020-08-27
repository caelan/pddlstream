(define (stream nightout)

  ; TODO: differentiate between types of cash
  (:stream s-cash
    :outputs (?c)
    :certified (and (wcash ?c) (cash ?c)))

  (:stream t-ge
    :inputs (?c1 ?c2)
    :domain (and (cash ?c1) (wcash ?c2))
    :certified (ge ?c1 ?c2))

  (:stream add
    :inputs (?c1 ?c2)
    :domain (and (cash ?c1) (wcash ?c2))
    :outputs (?c3)
    :certified (and (cash ?c3) (sum ?c1 ?c2 ?c3)
                    (ge ?c3 ?c1) (ge ?c3 ?c2)))

  (:stream subtract
    :inputs (?c3 ?c2)
    ;:domain (ge ?c3 ?c2) ; (and (cash ?c3) (cash ?c2))
    :domain (and (cash ?c3) (wcash ?c2))
    :outputs (?c1)
    :certified (and (cash ?c1) (sum ?c1 ?c2 ?c3)
                    (ge ?c3 ?c1)))

  ;(:function (Duration ?t)
  ;           (Traj ?t))
)