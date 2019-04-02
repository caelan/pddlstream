(define (stream namo)

  (:function (Cost ?r ?q1 ?q2)
    (and (Conf ?r ?q1) (Conf ?r ?q2))
  )

  ;(:stream sample-motion
  ;  :inputs (?v ?q1 ?q2)
  ;  :domain (and (Conf ?v ?q1) (Conf ?v ?q2))
  ;  :outputs (?t)
  ;  :certified (Motion ?v ?q1 ?t ?q2)
  ;)

  (:stream test-cfree-traj-pose
    :inputs (?r ?t ?b2 ?p2)
    :domain (and (Traj ?r ?t) (Pose ?b2 ?p2))
    :certified (CFreeTrajPose ?r ?t ?b2 ?p2)
  )
)
