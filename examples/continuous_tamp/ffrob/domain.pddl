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
    (Motion ?q1 ?t ?q2)
    (Placeable ?b ?s)
    (Traj ?t)
    (CFree ?r ?b1 ?g1 ?b2 ?p2)

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
    (UnsafeTraj ?r ?t)
    (Reachable ?r ?q)
  )
  (:functions
    (Dist ?q1 ?q2)
  )

  (:action move
    :parameters (?r ?q1 ?q2)
    :precondition (and (Robot ?r) (Conf ?q1) (Conf ?q2)
                       (AtConf ?r ?q1) ; (Reachable ?r ?q2)
                       (CanMove ?r))
    :effect (and (AtConf ?r ?q2)
                 (not (AtConf ?r ?q1)) (not (CanMove ?r))
                 (increase (total-cost) 1)))

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

  (:derived (UnsafeTraj ?r ?t)
    (exists (?b1 ?g1 ?b2 ?p2) (and (Robot ?r) (Traj ?t) (Grasp ?b1 ?p1) (Pose ?b2 ?p2)
                         (not (CFree ?r ?b1 ?g1 ?b2 ?p2))
                         (AtGrasp ?r ?b1 ?g1) (AtPose ?b2 ?p2)))
  )

  ; https://github.mit.edu/caelan/stripstream/blob/master/scripts/run_reachable.py
  ; examples/pybullet/namo/domain.pddl
  (:derived (Reachable ?r ?q2)
    (and (Robot ?r) (Conf ?q2)
        (or (AtConf ?r ?q2)
            (exists (?q1 ?t) (and (Motion ?q1 ?t ?q2) (Reachable ?r ?q1)
                                  ;(not (UnsafeTraj ?r ?t))
                                  ))))
  )
)