(define (stream namo)

  (:function (Cost ?r ?q1 ?q2)
    (and (Conf ?r ?q1) (Conf ?r ?q2))
  )
  (:stream sample-grasp
    :inputs (?r ?b)
    :domain (and (Robot ?r) (Body ?b))
    :outputs (?g)
    :certified (Grasp ?r ?b ?g)
  )

  (:stream compute-ik
    :inputs (?r ?b ?p ?g)
    :domain (and (Pose ?b ?p) (Grasp ?r ?b ?g))
    :outputs (?q)
    :certified (and (Kin ?r ?q ?b ?p ?g) (Conf ?r ?q))
  )

  (:stream compute-motion
    :inputs (?r ?q1 ?q2)
    :domain (and (Conf ?r ?q1) (Conf ?r ?q2))
    :outputs (?t)
    :certified (and (Motion ?r ?q1 ?q2 ?t) (Traj ?r ?t))
  )

  (:stream test-cfree-conf-pose
    :inputs (?r ?q ?b2 ?p2)
    :domain (and (Conf ?r ?q) (Pose ?b2 ?p2))
    :certified (CFreeConfPose ?r ?q ?b2 ?p2)
  )

  (:stream test-cfree-traj-pose
    :inputs (?r ?t ?b2 ?p2)
    :domain (and (Traj ?r ?t) (Pose ?b2 ?p2))
    :certified (CFreeTrajPose ?r ?t ?b2 ?p2)
  )

  (:stream test-reachable
    :inputs (?r ?q)
    :domain (Conf ?r ?q)
    :certified (Conf ?r ?q)
  )
)
