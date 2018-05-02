(define (stream pick-and-place)
  (:rule
    :parameters (?q ?p)
    :domain (Kin ?q ?p)
    :certified (and (Conf ?q) (Pose ?p))
  )
  (:rule
    :parameters (?p1 ?p2)
    :domain (AtPose ?b ?p)
    :certified (and (Block ?b) (Pose ?p))
  )

  (:function (Distance ?q1 ?q2)
    (and (Conf ?q1) (Conf ?q2))
  )
  (:predicate (Collision ?p1 ?p2)
    (and (Pose ?p1) (Pose ?p2))
  )

  (:stream sample-pose
    :inputs ()
    :domain ()
    :outputs (?p)
    :certified (and (Pose ?p))
  )
  (:stream inverse-kinematics
    :inputs (?p)
    :domain (Pose ?p)
    :outputs (?q)
    :certified (and (Conf ?q) (Kin ?q ?p))
  )
  (:stream collision-free
    :inputs (?p1 ?p2)
    :domain (and (Pose ?p1) (Pose ?p2))
    :outputs ()
    :certified (CFree ?p1 ?p2)
  )
)