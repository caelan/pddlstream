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

    (JustMoved)
    (Reachable ?r ?q2)
    (UnsafeTraj ?r ?t)
  )

  (:action move
    :parameters (?r ?q1 ?q2 ?t)
    :precondition (and (Motion ?r ?q1 ?q2 ?t)
                       (AtConf ?r ?q1) (not (UnsafeTraj ?r ?t)))
    :effect (and (AtConf ?r ?q2)
                 (not (AtConf ?r ?q1))
                 (increase (total-cost) (Cost ?r ?q1 ?q2)))
  )

  ; Similar to FFRob. See stripstream/scripts/run_reachable.py
  ;(:action move-teleport
  ;  :parameters (?r ?q1 ?q2)
  ;  :precondition (and (Conf ?r ?q1) (Conf ?r ?q2) ; (not (JustMoved))
  ;                     (AtConf ?r ?q1) (Reachable ?r ?q2) ) ; Technically don't need AtConf
  ;  :effect (and (AtConf ?r ?q2) (JustMoved)
  ;               (not (AtConf ?r ?q1))
  ;               (increase (total-cost) (Cost ?r ?q1 ?q2)))
  ;)

  (:action vaporize
    :parameters (?b ?p)
    :precondition (and (Pose ?b ?p)
                       (AtPose ?b ?p))
    :effect (not (AtPose ?b ?p))
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

  (:derived (UnsafeTraj ?r ?t)
    (exists (?b2 ?p2) (and (Traj ?r ?t) (Pose ?b2 ?p2)
                           (not (CFreeTrajPose ?r ?t ?b2 ?p2))
                           (AtPose ?b2 ?p2)))
  )

)