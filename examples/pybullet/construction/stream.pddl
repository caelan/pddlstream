(define (stream construction)
  (:stream test-cfree
    :inputs (?t ?e)
    :domain (and (Traj ?t) (Element ?e))
    :certified (CFree ?t ?e)
  )
)