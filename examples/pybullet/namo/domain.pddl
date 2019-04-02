(define (domain rovers)
  (:requirements :strips :equality)
  (:predicates
    (Robot ?r)
    (Conf ?r ?q)
    (Traj ?r ?t)
    (Motion ?r ?q1 ?q2 ?t)
    (CFreeTrajPose ?r ?t ?b2 ?p2)

    (Body ?b)
    (Pose ?b ?p)

    (AtConf ?r ?q)
    (AtPose ?b ?p)

    (UnsafeTraj ?r ?t)
  )

  (:action move
    :parameters (?r ?q1 ?q2 ?t)
    :precondition (and (Motion ?r ?q1 ?q2 ?t)
                       (AtConf ?r ?q1) (not (UnsafeTraj ?r ?t)))
    :effect (and (AtConf ?r ?q2)
                 (not (AtConf ?r ?q1)))
  )

  (:action vaporize
    :parameters (?b ?p)
    :precondition (and (Pose ?b ?p)
                       (AtPose ?b ?p))
    :effect (not (AtPose ?b ?p))
  )

  (:derived (UnsafeTraj ?r ?t)
    (exists (?b2 ?p2) (and (Traj ?r ?t) (Pose ?b2 ?p2)
                           (not (CFreeTrajPose ?r ?t ?b2 ?p2))
                           (AtPose ?b2 ?p2)))
  )

)