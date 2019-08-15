(define (stream exogenous)
  (:stream inv-kin
    :inputs (?p)
    :domain (and (Pose ?p) (Observable ?p)) ; Comment out Observable
    :outputs (?q)
    :certified (and (Conf ?q) (Kin ?q ?p))
  )
  (:stream motion
    :inputs (?q1 ?q2)
    :domain (and (Conf ?q1) (Conf ?q2))
    :outputs (?t)
    :certified (Motion ?q1 ?t ?q2)
  )
)