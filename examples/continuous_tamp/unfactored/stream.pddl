(define (stream pick-and-place)
  (:stream sample-pose
    :inputs (?b ?r)
    :domain (Placeable ?b ?r)
    :outputs (?p)
    :certified (and (Pose ?b ?p) (Contained ?b ?p ?r))
  )
  (:stream inverse-kinematics
    :inputs (?b ?p)
    :domain (Pose ?b ?p)
    :outputs (?q)
    :certified (and (Conf ?q) (Kin ?b ?q ?p))
  )
  (:stream plan-motion
    :inputs (?q1 ?q2)
    :domain (and (Conf ?q1) (Conf ?q2))
    :outputs (?t)
    :certified (and (Traj ?t) (Motion ?q1 ?t ?q2))
  )

  (:stream forward-move ; Fluents in domain to make easier to ground
    :inputs (?s1 ?q1 ?t ?q2)
    :domain (and (State ?s1) (Motion ?q1 ?t ?q2))
    :outputs (?s2)
    :certified (and (State ?s2) (Move ?s1 ?q1 ?t ?q2 ?s2)
                    (AtConf ?s2 ?q2)) ; Unchanged variables?
  )
  (:stream forward-pick
    :inputs (?s1 ?b ?q ?p)
    :domain (and (State ?s1) (Kin ?b ?q ?p))
    :outputs (?s2)
    :certified (and (State ?s2) (Pick ?s1 ?b ?q ?p ?s2)
                    (Holding ?s2 ?b) (AtConf ?s2 ?q))
  )
  (:stream forward-place
    :inputs (?s1 ?b ?q ?p)
    :domain (and (State ?s1) (Kin ?b ?q ?p)) ; Add to preconditions?
    :outputs (?s2)
    :certified (and (State ?s2) (Place ?s1 ?b ?q ?p ?s2)
                    (AtPose ?s2 ?b ?p) (HandEmpty ?s2) (AtConf ?s2 ?q))
  )
  (:stream test-goal
    :inputs (?s)
    :domain (State ?s)
    :outputs ()
    :certified (SatisfiesGoal ?s)
  )
  ; Wild stream that maps to fluents?
)