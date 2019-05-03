(define (domain continuous-tamp)
  (:requirements :strips :equality)
  (:predicates
    ; Static predicates
    (Robot ?r)
    (Block ?b)
    (Region ?s)
    (Pose ?b ?p)
    (Grasp ?b ?g)
    (Conf ?q)
    (Traj ?t)
    (Contain ?b ?p ?s)
    (Kin ?b ?q ?p ?g)
    (Motion ?q1 ?t ?q2)
    (CFree ?b1 ?p1 ?b2 ?p2)
    (Placeable ?b ?s)
    (PoseCollision ?b1 ?p1 ?b2 ?p2)
    (TrajCollision ?t ?b2 ?p2)

    ; Fluent predicates
    (AtPose ?b ?p)
    (AtGrasp ?b ?g)
    (AtConf ?q)
    (Holding ?b)
    (HandEmpty)
    (CanMove)

    ; Derived predicates
    (In ?b ?s)
    (UnsafePose ?b ?p)
    (UnsafeTraj ?t)
  )
  (:functions
    (Dist ?q1 ?q2)
  )

  (:action move
    :parameters (?r ?q1 ?t ?q2)
    :precondition (and (Robot ?r) (Motion ?q1 ?t ?q2)
                       (AtConf ?q1) (CanMove)) ; (not (UnsafeTraj ?t)))
    :effect (and (AtConf ?q2)
                 (not (AtConf ?q1)) (not (CanMove))
                 (increase (total-cost) (Dist ?q1 ?q2))))

  (:action pick
    :parameters (?r ?b ?p ?g ?q)
    :precondition (and (Robot ?r) (Kin ?b ?q ?p ?g)
                       (AtConf ?q) (AtPose ?b ?p) (HandEmpty))
    :effect (and (AtGrasp ?b ?g) (CanMove)
                 (not (AtPose ?b ?p)) (not (HandEmpty))
                 (increase (total-cost) 10)))

  (:action place
    :parameters (?r ?b ?p ?g ?q)
    :precondition (and (Robot ?r) (Kin ?b ?q ?p ?g)
                       (AtConf ?q) (AtGrasp ?b ?g)
                       (not (UnsafePose ?b ?p))
                       (forall (?b2 ?p2) ; TODO: makes incremental slow
                         (imply (and (Pose ?b2 ?p2) (AtPose ?b2 ?p2))
                                (CFree ?b ?p ?b2 ?p2)))
                  )
    :effect (and (AtPose ?b ?p) (HandEmpty) (CanMove)
                 (not (AtGrasp ?b ?g))
                 (increase (total-cost) 10))
  )

  (:derived (In ?b ?s)
    (exists (?p) (and (Contain ?b ?p ?s)
                      (AtPose ?b ?p))))
  (:derived (Holding ?b)
    (exists (?g) (and (Grasp ?b ?g)
                      (AtGrasp ?b ?g))))
  ;(:derived (UnsafePose ?b1 ?p1)
  ;  (exists (?b2 ?p2) (and (Pose ?b1 ?p1) (Pose ?b2 ?p2)
  ;                         (AtPose ?b2 ?p2)
  ;                         (not (CFree ?b1 ?p1 ?b2 ?p2))
  ;                     )))
)