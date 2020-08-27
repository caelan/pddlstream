(define (stream nightout)

  (:stream s-cash
    :outputs (?c)
    :certified (and (wcash ?c) (cash ?c)))

  (:stream t-ge
    :inputs (?c1 ?c2)
    ;:domain (and (cash ?c1) (cash ?c2))
    ;:domain (and (pcash ?c1) (pcash ?c2))
    :domain (and (pcash ?c1) (tcash ?c2))
    :certified (ge ?c1 ?c2))

  ; TODO: condition on the person and machine
  (:stream add
    :inputs (?c1 ?c2)
    :domain (and (pcash ?c1) (wcash ?c2))
    :outputs (?c3)
    :certified (and (pcash ?c3) (cash ?c3) (sum ?c1 ?c2 ?c3)
                    (ge ?c3 ?c1))) ; (ge ?c3 ?c2)

  (:stream subtract
    :inputs (?c3 ?c2)
    ;:domain (ge ?c3 ?c2) ; (and (cash ?c3) (cash ?c2))
    :domain (and (mcash ?c3) (wcash ?c2))
    :outputs (?c1)
    :certified (and (mcash ?c1) (cash ?c1) (sum ?c1 ?c2 ?c3))) ; (ge ?c3 ?c1)))

  ;(:function (Duration ?t)
  ;           (Traj ?t))
)