(define (stream construction)
  (:stream test-cfree
    :inputs (?t ?e)
    :domain (and (Traj ?t) (Element ?e))
    :certified (CFree ?t ?e)
  )
  (:stream sample-print
    :inputs (?n ?e)
    :domain (StartNode ?n ?e)
    :outputs (?t)
    :certified (and (PrintAction ?n ?e ?t) (Traj ?t))
  )
)