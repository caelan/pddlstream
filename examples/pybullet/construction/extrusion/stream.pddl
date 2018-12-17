(define (stream construction)
  ;(:stream test-cfree
  ;  :inputs (?t ?e)
  ;  :domain (and (Traj ?t) (Element ?e))
  ;  :certified (CFree ?t ?e)
  ;)
  ; (:stream sample-print
  (:wild-stream sample-print
    :inputs (?n ?e)
    :domain (StartNode ?n ?e)
    ; :fluents (Printed)
    :outputs (?t)
    :certified (and (PrintAction ?n ?e ?t) (Traj ?t))
  )
  (:stream test-stiffness
   :fluents (Printed)
   :certified (Stiff)
  )
)