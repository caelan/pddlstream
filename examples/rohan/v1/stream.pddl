(define (stream tom)
  (:stream physics
    :inputs (?s ?a)
    :domain (and (State ?s) (Action ?a))
    :outputs (?ns)
    :certified (and (IsValidPhysics ?s ?a ?ns) (State ?ns)))
  (:stream positive-actions
    :outputs (?a)
    :certified (Action ?a))
  (:stream negative-actions
    :outputs (?a)
    :certified (Action ?a))
)
