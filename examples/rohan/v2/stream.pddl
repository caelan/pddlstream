(define (stream tom)
  (:stream physics
    :inputs (?s ?a)
    :domain (and (State ?s) (Action ?a))
    :outputs (?ns)
    :certified (and (IsValidPhysics ?s ?a ?ns) (State ?ns)))
  (:stream positive-actions
    :outputs (?a)
    :certified (Action ?a))
  (:stream check-goal
    :inputs (?s)
    :domain (State ?s)
    :outputs ()
    :certified (IsGoalState ?s))
)
