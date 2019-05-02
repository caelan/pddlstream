(define (domain temporal-tamp)
    (:requirements :equality :typing :durative-actions)
	; (:types block)
    (:predicates
        ; Static predicates
        (Block ?b)
        (Region ?r)
        (Pose ?b ?p)
        (Grasp ?b ?g)
        (Conf ?q)
        (Traj ?t)
        (Contain ?b ?p ?r)
        (Kin ?b ?q ?p ?g)
        (Motion ?q1 ?t ?q2)
        (CFree ?b1 ?p1 ?b2 ?p2)
        (Placeable ?b ?r)
        (PoseCollision ?b1 ?p1 ?b2 ?p2)
        (TrajCollision ?t ?b2 ?p2)

        ; Fluent predicates
        (AtPose ?b ?p)
        (AtGrasp ?b ?g)
        (AtConf ?q)
        (HandEmpty)
        (CanMove)

        ; Derived predicates
        (In ?b ?r) ; TFLAP redeclared predicate
        (Holding ?b)
        (UnsafePose ?b ?p)
        (UnsafeTraj ?t)
    )
    (:functions
        (Dist ?q1 ?q2)
        (total-cost)
        (Fuel)
    )

	(:durative-action move
		:parameters (?q1 ?t ?q2)
		:duration (= ?duration 1)
		:condition (and
			(at start (and (Conf ?q1) (Conf ?q2)))
			(at start (AtConf ?q1))
		)
		:effect (and
			(at start (not (AtConf ?q1)))
			(at end (AtConf ?q2))
			; (at end (increase (total-cost) 1)) ; Many temporal planners don't support costs
			; (at end (decrease (Fuel) 1)) ; Numeric effects not currently supported
		)
	)
	(:durative-action pick
		:parameters (?b ?p ?g ?q)
		:duration (= ?duration 1)
		:condition (and
			(at start (Kin ?b ?q ?p ?g))
			(at start (AtPose ?b ?p))
			(at start (HandEmpty))
			(over all (AtConf ?q))
		)
		:effect (and
			(at start (not (AtPose ?b ?p)))
			(at start (not (HandEmpty)))
			(at end (AtGrasp ?b ?g))
		)
	)
	(:durative-action place ; Could also just make the region an arg
		:parameters (?b ?p ?g ?q)
		:duration (= ?duration 1)
		:condition (and
			(at start (Kin ?b ?q ?p ?g))
			(at start (AtGrasp ?b ?g))
			(over all (AtConf ?q))
		)
		:effect (and
		    (at start (not (AtGrasp ?b ?g)))
			(at end (AtPose ?b ?p))
			(at end (HandEmpty))
			; (forall (?r) (at end (In ?b ?r)))
	        (when (at end (HandEmpty)) (at end (Holding ?b)))
		)
	)

    ; TFLAP doesn't support derived predicates
    ;(:derived (In ?b ?r)
    ;    (exists (?p) (and (Contain ?b ?p ?r)
    ;                      (AtPose ?b ?p))))
    ;
    ;(:derived (Holding ?b)
    ;    (exists (?g) (and (Grasp ?b ?g)
    ;                      (AtGrasp ?b ?g))))
)