(define (domain rovers)
  (:requirements :equality :durative-actions :numeric-fluents :derived-predicates :conditional-effects)
  ; (:contants r0 r1)
  (:predicates
    (Robot ?r)
    (Conf ?q)
    (Traj ?t)
    (Motion ?q1 ?t ?q2)
    (CFreeTrajTraj ?t1 ?t2)

    (Safe)
    (AtConf ?r ?q)
    (OnTraj ?r ?t)
    (UnsafeTraj ?r ?t)
  )

  (:durative-action move
		:parameters (?r ?q1 ?t ?q2)
		:duration (= ?duration 1)
		:condition (and
			(at start (and (Robot ?r) (Motion ?q1 ?t ?q2)))
			(at start (AtConf ?r ?q1))

      (at start (Safe))
      (over all (Safe))
			(at end (Safe))

      (at start (not (Unsafe ?r ?t)))
      (over all (not (Unsafe ?r ?t)))
			(at end (not (Unsafe ?r ?t)))
		)
		:effect (and
			(at start (not (AtConf ?r ?q1)))
      (at start (OnTraj ?r ?t))
      (at end (not (OnTraj ?r ?t)))
			(at end (AtConf ?r ?q2))
      ; (forall (?r2 ?t2) (when (over all (OnTraj ?r2 ?t2)) (at end (not (Safe)))))
		)
	)

  (:derived (UnsafeTraj ?r1 ?t1)
    (exists (?r2 ?t2) (and (Robot ?r1) (Traj ?t1) (Robot ?r2) (Traj ?t2)
                           (not (= ?r1 ?r2)) (not (CFreeTrajTraj ?t1 ?t2))
                           (OnTraj ?r2 ?t2)))
  )
)