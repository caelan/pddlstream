(define (stream pick-and-place)
  (:stream sample-pose
    :inputs (?b ?s)
    :domain (Placeable ?b ?s)
    :outputs (?p)
    :certified (and (Pose ?b ?p) (Contained ?b ?p ?s))
  )
  (:stream inverse-kinematics
    :inputs (?b ?p ?g)
    :domain (and (Pose ?b ?p) (Grasp ?b ?g))
    :outputs (?q)
    :certified (and (Conf ?q) (Kin ?b ?q ?p ?g))
  )
  (:stream plan-motion
    :inputs (?q1 ?q2)
    :domain (and (Conf ?q1) (Conf ?q2))
    :outputs (?t)
    :certified (and (Traj ?t) (Motion ?q1 ?t ?q2))
  )

  (:stream forward-move ; Fluents in domain to make easier to ground
    :inputs (?s1 ?r ?q1 ?t ?q2)
    :domain (and (State ?s1) (Robot ?r) (Motion ?q1 ?t ?q2))
    :outputs (?s2)
    :certified (and (State ?s2) (Move ?s1 ?r ?q1 ?t ?q2 ?s2)
                    ; (AtConf ?s2 ?r ?q2)
               ) ; Unchanged variables?
  )
  (:stream forward-pick
    :inputs (?s1 ?r ?b ?q ?p ?g)
    :domain (and (State ?s1) (Robot ?r) (Kin ?b ?q ?p ?g))
    :outputs (?s2)
    :certified (and (State ?s2) (Pick ?s1 ?r ?b ?q ?p ?g ?s2)
                    ; (Holding ?s2 ?r ?b) (AtConf ?s2 ?r ?q)
               )
  )
  (:stream forward-place
    :inputs (?s1 ?r ?b ?q ?p ?g)
    :domain (and (State ?s1) (Robot ?r) (Kin ?b ?q ?p ?g)) ; Add to preconditions?
    :outputs (?s2)
    :certified (and (State ?s2) (Place ?s1 ?r ?b ?q ?p ?g ?s2)
                    ; (AtPose ?s2 ?b ?p) (HandEmpty ?s2 ?r) (AtConf ?s2 ?r ?q)
               )
  )
  (:stream test-goal
    :inputs (?s)
    :domain (State ?s)
    :outputs ()
    :certified (SatisfiesGoal ?s)
  )
  ; Wild stream that maps to fluents?
)