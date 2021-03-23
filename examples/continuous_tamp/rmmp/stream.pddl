(define (stream continuous-tamp)

  ; Could also test
  ;(:stream t-adjacent
  ;  :inputs (?m1 ?m2)
  ;  :domain (and (Mode ?m1) (Mode ?m2))
  ;  :certified (Adjacent ?m1 ?m2))

  (:stream s-forward
    :inputs (?m1)
    :domain (Mode ?m1)
    :outputs (?m2)
    :certified (and (Mode ?m2) (Adjacent ?m1 ?m2))) ; (Adjacent ?m2 ?m1)

  (:stream s-intersection
    :inputs (?m1 ?m2)
    :domain (Adjacent ?m1 ?m2)
    :outputs (?q)
    :certified (and (Conf ?m1 ?q) (Conf ?m2 ?q)
                    (Switch ?m1 ?m2 ?q)))

  ;(:stream s-conf
  ;  :inputs (?m)
  ;  :domain (Mode ?m)
  ;  :outputs (?q)
  ;  :certified (Conf ?m ?q))

  (:stream s-connection
    :inputs (?m ?q1 ?q2)
    :domain (and (Conf ?m ?q1) (Conf ?m ?q2))
    :outputs (?t)
    :certified (Motion ?m ?q1 ?t ?q2))

  (:stream t-goal
    :inputs (?m ?q)
    :domain (Conf ?m ?q)
    :certified (GoalState ?m ?q))

  ;(:function (Duration ?t)
  ;           (Traj ?t))
)