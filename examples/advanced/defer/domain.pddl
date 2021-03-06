(define (domain pick-and-place)
  (:requirements :strips)
  (:predicates
    (Conf ?q)
    (Block ?b)
    (Pose ?p)
    (Contain ?p ?r)
    (Kin ?q ?p)
    (Motion ?q1 ?t ?q2)

    (AtPose ?p ?q)
    (AtConf ?q)
    (Holding ?b)
    (HandEmpty)

    (In ?b ?r)
  )
  (:action move
    :parameters (?q1 ?t ?q2)
    :precondition (and (Motion ?q1 ?t ?q2)
                       (AtConf ?q1))
    :effect (and (AtConf ?q2)
                 (not (AtConf ?q1)))
  )
  (:action pick
    :parameters (?b ?p ?q)
    :precondition (and (Block ?b) (Kin ?q ?p)
                       (AtConf ?q) (AtPose ?b ?p) (HandEmpty))
    :effect (and (Holding ?b)
                 (not (AtPose ?b ?p)) (not (HandEmpty)))
  )
  (:action place
    :parameters (?b ?p ?q)
    :precondition (and (Block ?b) (Kin ?q ?p)
                       (AtConf ?q) (Holding ?b))
    :effect (and (AtPose ?b ?p) (HandEmpty)
                 (not (Holding ?b)))
  )

  (:derived (In ?b ?r)
    (exists (?p) (and (Block ?b) (Contain ?p ?r)
                      (AtPose ?b ?p))))
)