(define (domain rovers)
  (:requirements :strips :equality)
  (:predicates
    (Robot ?r)
    (Conf ?r ?q)
    (Traj ?r ?t)
    (Kin ?r ?q ?b ?p ?g)
    (Motion ?r ?q1 ?q2 ?t)
    (CFreeConfPose ?r ?q ?b2 ?p2)
    (CFreeTrajPose ?r ?t ?b2 ?p2)

    (Body ?b)
    (Pose ?b ?p)
    (Grasp ?r ?b ?g)

    (AtConf ?r ?q)
    (AtPose ?b ?p)
    (AtGrasp ?r ?b ?g)
    (Free ?r)

    (JustMoved)
    (Reachable ?r ?q2)
    (Holding ?r ?b)
    (UnsafeConf ?r ?q)
    (UnsafeTraj ?r ?t)
  )

  (:action move
    :parameters (?r ?q1 ?q2 ?t)
    :precondition (and (Motion ?r ?q1 ?q2 ?t)
                       (AtConf ?r ?q1)
                       (not (UnsafeConf ?r ?q2)) (not (UnsafeTraj ?r ?t)))
    :effect (and (AtConf ?r ?q2)
                 (not (AtConf ?r ?q1))
                 (increase (total-cost) (Cost ?r ?q1 ?q2)))
  )

  ; Similar to FFRob. See stripstream/scripts/run_reachable.py
  (:action move-teleport
    :parameters (?r ?q1 ?q2)
    :precondition (and (Conf ?r ?q1) (Conf ?r ?q2) ; (not (JustMoved))
                       (or (Reachable ?r ?q1) (Reachable ?r ?q2)) ; Need both
                       (AtConf ?r ?q1))
    :effect (and (AtConf ?r ?q2) ; (JustMoved) ; TODO: some sort of problem with JustMoved
                 (not (AtConf ?r ?q1))
                 (increase (total-cost) (Cost ?r ?q1 ?q2)))
  )

  ;(:action vaporize
  ;  :parameters (?b ?p)
  ;  :precondition (and (Pose ?b ?p)
  ;                     (AtPose ?b ?p))
  ;  :effect (and (not (AtPose ?b ?p))
  ;               (increase (total-cost) 1))
  ;)
  ;(:action vaporize
  ;  :parameters (?r ?q ?b ?p ?g)
  ;  :precondition (and (Kin ?r ?q ?b ?p ?g)
  ;                     (Free ?r) (AtConf ?r ?q) (AtPose ?b ?p))
  ;  :effect (and (not (AtPose ?b ?p))
  ;               (increase (total-cost) 1))
  ;)

  (:action pick
    :parameters (?r ?q ?b ?p ?g)
    :precondition (and (Kin ?r ?q ?b ?p ?g)
                       (Free ?r) (AtConf ?r ?q) (AtPose ?b ?p))
    :effect (and (AtGrasp ?r ?b ?g)
                 (not (AtPose ?b ?p)) ; (not (JustMoved))
                 (increase (total-cost) 1))
  )

  (:derived (Holding ?r ?b)
    (exists (?g) (and (Grasp ?r ?b ?g)
                      (AtGrasp ?r ?b ?g)))
  )

  (:derived (UnsafeConf ?r ?q)
    (exists (?b2 ?p2) (and (Conf ?r ?q) (Pose ?b2 ?p2)
                           (not (CFreeConfPose ?r ?q ?b2 ?p2))
                           (AtPose ?b2 ?p2)))
  )
  (:derived (UnsafeTraj ?r ?t)
    (exists (?b2 ?p2) (and (Traj ?r ?t) (Pose ?b2 ?p2)
                           (not (CFreeTrajPose ?r ?t ?b2 ?p2))
                           (AtPose ?b2 ?p2)))
  )

  ;(:derived (Reachable ?r ?q2)
  ;  (and (Conf ?r ?q2)
  ;      (or (AtConf ?r ?q2)
  ;          (exists (?q1 ?t) (and (Motion ?r ?q1 ?q2 ?t)
  ;                                ; TODO: current don't support negated predicates in axioms
  ;                                ; (extraction of an axiom plan fails to consider them)
  ;                                (not (UnsafeTraj ?r ?t))
  ;                                (Reachable ?r ?q1)
  ;                           ))))
  ;)
)