(define (stream exogenous)
  (:stream inv-kin
    :inputs (?p)
    :domain (and (Pose ?p) (Observable ?p))
    :outputs (?q)
    :certified (and (Conf ?q) (Kin ?q ?p))
  )
)