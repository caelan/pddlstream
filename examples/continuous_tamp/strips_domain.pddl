(define (domain continuous-tamp)
  (:requirements :strips :equality)
  (:predicates
    ; Static predicates
    (Robot ?r)
    (Block ?b)
    (Region ?s)
    (Stove ?s)
    (Placeable ?b ?s)

    (Pose ?b ?p)
    (Grasp ?b ?g)
    (Conf ?q)
    (Traj ?t)
    (Contain ?b ?p ?s)
    (Kin ?b ?q ?p ?g)
    (Motion ?q1 ?t ?q2)
    (CFree ?b1 ?p1 ?b2 ?p2)

    ; Fluent predicates
    (AtPose ?b ?p)
    (AtGrasp ?r ?b ?g)
    (AtConf ?r ?q)
    (HandEmpty ?r)
    (CanMove ?r)
    (Cooked ?b)

    ; Derived predicates
    (Holding ?r ?b)
    (In ?b ?s)
    (UnsafePose ?b1 ?p1 ?b2)
  )
  (:functions
    (Cost)
    (Dist ?q1 ?q2)
  )

  (:action move
    :parameters (?r ?q1 ?t ?q2)
    :precondition (and (Robot ?r) (Motion ?q1 ?t ?q2)
                       (AtConf ?r ?q1) (CanMove ?r)
                       (forall (?b1 ?p1 ?b2) (imply (and (Pose ?b1 ?p1) (Block ?b2))
                                                    (not (UnsafePose ?b1 ?p1 ?b2))))
                  )
    :effect (and (AtConf ?r ?q2)
                 (not (AtConf ?r ?q1)) (not (CanMove ?r))
                 (increase (total-cost) (Dist ?q1 ?q2))))

  (:action pick
    :parameters (?r ?b ?p ?g ?q)
    :precondition (and (Robot ?r) (Kin ?b ?q ?p ?g) ; (Contain ?b ?p ?s) (In ?b ?s)
                       (AtConf ?r ?q) (AtPose ?b ?p) (HandEmpty ?r)
                  )
    :effect (and (AtGrasp ?r ?b ?g) (Holding ?r ?b) (CanMove ?r)
                 (not (AtPose ?b ?p)) (not (HandEmpty ?r))
                 (increase (total-cost) (Cost))))

  (:action place
    :parameters (?r ?b ?p ?g ?q ?s)
    :precondition (and (Robot ?r) (Kin ?b ?q ?p ?g) (Contain ?b ?p ?s)
                       ; TODO: opposite version where the search pays before using (like with axioms)
                       (AtConf ?r ?q) (AtGrasp ?r ?b ?g)
                       ; TODO: recreate a precondition version
                       ;(forall (?b2 ?p2) ; TODO: makes incremental slow
                       ;  (imply (and (Pose ?b2 ?p2) (AtPose ?b2 ?p2))
                       ;         (CFree ?b ?p ?b2 ?p2)))
                  )
    :effect (and (AtPose ?b ?p) (In ?b ?s) (HandEmpty ?r) (CanMove ?r)
                 (not (AtGrasp ?r ?b ?g)) (not (Holding ?r ?b))
                 ; TODO: condition that negates all (not (UnsafePose ?b1 ?p1 ?b2))
                 (forall (?b2) (when (and (Block ?b2) (not (= ?b ?b2)))
                                     (UnsafePose ?b ?p ?b2)))
                 (increase (total-cost) (Cost))))

  (:action cook
    :parameters (?b ?s)
    :precondition (and (Placeable ?b ?s) (Stove ?s)
                       (In ?b ?s))
    :effect (and (Cooked ?b)
                 (increase (total-cost) (Cost))))

  (:action prove-safe ; TODO: maybe just use the subsequent state
    ; TODO: impose partial orders on objects to reduce branching factor
    ; TODO: condition (UnsafePose ?b1 ?p1 ?b2) before must not hold
    :parameters (?b1 ?p1 ?b2 ?p2)
    :precondition (and (Pose ?b1 ?p1) (Pose ?b2 ?p2)
                       (CFree ?b1 ?p1 ?b2 ?p2)
                       (AtPose ?b2 ?p2) (UnsafePose ?b1 ?p1 ?b2)
    )
    :effect (not (UnsafePose ?b1 ?p1 ?b2)))
)