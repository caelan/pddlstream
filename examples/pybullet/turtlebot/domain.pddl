(define (domain rovers)
  (:requirements :equality :durative-actions :numeric-fluents :derived-predicates :conditional-effects)
  (:predicates
    (Robot ?r)
    (Conf ?r ?q)
    (Traj ?r ?t)
    (Motion ?r ?q1 ?q2 ?t)
    (CFreeTrajPose ?r ?t ?b2 ?p2)

    (AtConf ?r ?q)

    (UnsafeTraj ?r ?t)
  )

  (:durative-action move
		:parameters (?r ?q1 ?t ?q2)
		:duration (= ?duration 1)
		:condition (and
			(at start (and (Robot ?r) (Motion ?q1 ?t ?q2)))
			(at start (AtConf ?r ?q1))
			;(over all (Safe))
		)
		:effect (and
			(at start (not (AtConf ?r ?q1)))
			(at end (AtConf ?r ?q2))
		)
	)

  ;(:derived (UnsafeTraj ?r ?t)
  ;  (exists (?b2 ?p2) (and (Traj ?r ?t) (Pose ?b2 ?p2)
  ;                         (not (CFreeTrajPose ?r ?t ?b2 ?p2))
  ;                         (AtPose ?b2 ?p2)))
  ;)
)