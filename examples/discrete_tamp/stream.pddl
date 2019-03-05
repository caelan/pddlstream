(define (stream discrete-tamp)
  (:rule
    :inputs (?q ?p)
    :domain (Kin ?q ?p)
    :certified (and (Conf ?q) (Pose ?p))
  )
  ;(:rule
  ;  :inputs (?b ?p)
  ;  :domain (AtPose ?b ?p) ; This is a fluent
  ;  :certified (and (Block ?b) (Pose ?p))
  ;)

  (:function (Distance ?q1 ?q2)
    (and (Conf ?q1) (Conf ?q2))
  )

  (:stream sample-pose
    :outputs (?p)
    :certified (Pose ?p)
  )
  (:stream inverse-kinematics
    :inputs (?p)
    :domain (Pose ?p)
    :outputs (?q)
    :certified (Kin ?q ?p)
  )
  (:stream test-cfree
    :inputs (?p1 ?p2)
    :domain (and (Pose ?p1) (Pose ?p2))
    :certified (CFree ?p1 ?p2)
  )
)