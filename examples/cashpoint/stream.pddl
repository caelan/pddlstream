(define (stream nightout)

  (:stream s-cash
    :outputs (?wc)
    :certified (and (wcash ?wc) (cash ?wc)))

  (:stream t-ge
    :inputs (?pc ?tc)
    ;:domain (and (cash ?pc) (cash ?tc))
    ;:domain (and (pcash ?pc) (pcash ?tc))
    :domain (and (pcash ?pc) (tcash ?tc))
    :certified (ge ?pc ?tc))

  ; TODO: condition on the person and machine
  ;(:stream add
  ;  :inputs (?pc1 ?wc)
  ;  :domain (and (pcash ?pc1) (wcash ?wc))
  ;  :outputs (?pc2)
  ;  :certified (and (pcash ?pc2) (cash ?pc2)
  ;                  (sum ?pc1 ?wc ?pc2)
  ;                  (ge ?pc2 ?pc1))) ; (ge ?pc2 ?wc)

  ;(:stream subtract
  ;  :inputs (?mc2 ?wc)
  ;  ;:domain (ge ?mc2 ?wc) ; (and (cash ?mc2) (cash ?wc))
  ;  :domain (and (mcash ?mc2) (wcash ?wc))
  ;  :outputs (?mc1)
  ;  :certified (and (mcash ?mc1) (cash ?mc1)
  ;             (sum ?mc1 ?wc ?mc2))) ; (ge ?mc2 ?mc1)))

  (:stream withdraw
    :inputs (?wcash ?pcash1 ?mcash1) ; Could sample ?wcash internally
    :domain (and (wcash ?wcash) (pcash ?pcash1) (mcash ?mcash1))
    :outputs (?pcash2 ?mcash2)
    :certified (and (pcash ?pcash2) (mcash ?mcash2)
                    (withdraw ?wcash ?pcash1 ?pcash2 ?mcash1 ?mcash2)))

  (:function (withdrawcost ?c)
             (wcash ?c))
)