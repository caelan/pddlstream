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
    (Contain ?b ?p ?s)
    (Kin ?b ?q ?p ?g)
    (Reachable ?q1 ?q2)
    (Motion ?q1 ?t ?q2)
    (Placeable ?b ?s)

    ; Fluent predicates
    (AtPose ?b ?p)
    (AtGrasp ?r ?b ?g)
    (AtConf ?r ?q)
    (Holding ?r ?b)
    (HandEmpty ?r)
    (CanMove ?r)

    ; Derived predicates
    (Holding ?r ?b)
    (In ?b ?s)
  )
  (:functions
    (Dist ?q1 ?q2)
  )

  (:action move
    ;:parameters (?r ?q1 ?q2)
    ;:precondition (and (Robot ?r) (Reachable ?q1 ?q2)
    :parameters (?r ?q1 ?t ?q2)
    :precondition (and (Robot ?r) (Motion ?q1 ?t ?q2)
                        (AtConf ?r ?q1) (CanMove ?r)) ; Works for both :parameters
    :effect (and (AtConf ?r ?q2)
                 (not (AtConf ?r ?q1)) (not (CanMove ?r))
                 (increase (total-cost) 1)))
                 ;(increase (total-cost) (Dist ?q1 ?q2))))

  (:action pick
    :parameters (?r ?b ?p ?g ?q)
    :precondition (and (Robot ?r) (Kin ?b ?q ?p ?g)
                       (AtConf ?r ?q) (AtPose ?b ?p) (HandEmpty ?r))
    :effect (and (AtGrasp ?r ?b ?g) (CanMove ?r)
                 (not (AtPose ?b ?p)) (not (HandEmpty ?r))
                 (increase (total-cost) 1)))

  (:action place
    :parameters (?r ?b ?p ?g ?q)
    :precondition (and (Robot ?r) (Kin ?b ?q ?p ?g)
                       (AtConf ?r ?q) (AtGrasp ?r ?b ?g))
    :effect (and (AtPose ?b ?p) (HandEmpty ?r) (CanMove ?r)
                 (not (AtGrasp ?r ?b ?g))
                 (increase (total-cost) 1))
  )

  (:derived (In ?b ?s)
    (exists (?p) (and (Contain ?b ?p ?s)
                      (AtPose ?b ?p))))
  (:derived (Holding ?r ?b)
    (exists (?g) (and (Robot ?r) (Grasp ?b ?g)
                      (AtGrasp ?r ?b ?g))))
)