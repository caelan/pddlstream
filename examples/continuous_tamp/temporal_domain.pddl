(define (domain temporal-tamp)
    (:requirements :typing :durative-actions :numeric-fluents :derived-predicates :conditional-effects) ; :equality :action-costs
	; (:types block)
	; (:constants )
    (:predicates
        ; Static predicates
        (Robot ?r)
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
        (AtGrasp ?r ?b ?g)
        (AtConf ?r ?q)
        (HandEmpty ?r)
        (CanMove ?r)

        (Stove ?r)
        (Cooked ?b)

        ; Derived predicates
        (In ?b ?r) ; TFLAP redeclared predicate
        (Holding ?r ?b)
        (UnsafePose ?b ?p)
        (UnsafeTraj ?t)
    )
    (:functions
        (Dist ?q1 ?q2)
        (total-cost)
        ; (Fuel)
    )

	(:durative-action move
		:parameters (?r ?q1 ?t ?q2)
		:duration (= ?duration 1)
		:condition (and
			(at start (and (Robot ?r) (Conf ?q1) (Conf ?q2)))
			(at start (AtConf ?r ?q1))
		)
		:effect (and
			(at start (not (AtConf ?r ?q1)))
			(at end (AtConf ?r ?q2))
			; (at end (increase (total-cost) 1)) ; Many temporal planners don't support costs
			; (at end (decrease (Fuel) 1)) ; Numeric effects not currently supported
		)
	)
	(:durative-action pick
		:parameters (?r ?b ?p ?g ?q)
		:duration (= ?duration 1)
		:condition (and
			(at start (Robot ?r))
			(at start (Kin ?b ?q ?p ?g))
			(at start (AtPose ?b ?p))
			(at start (HandEmpty ?r))
			(over all (AtConf ?r ?q))
		)
		:effect (and
			(at start (not (AtPose ?b ?p)))
			(at start (not (HandEmpty ?r)))
			(at end (AtGrasp ?r ?b ?g))
		)
	)
	(:durative-action place ; Could also just make the region an arg
		:parameters (?r ?b ?p ?g ?q)
		:duration (= ?duration 1)
		:condition (and
		    (at start (Robot ?r))
			(at start (Kin ?b ?q ?p ?g))
			(at start (AtGrasp ?r ?b ?g))
			(over all (AtConf ?r ?q))
		)
		:effect (and
		    (at start (not (AtGrasp ?r ?b ?g)))
			(at end (AtPose ?b ?p))
			(at end (HandEmpty ?r))
			; (forall (?r) (when (at start (Contain ?b ?p ?r)) (at end (In ?b ?r))))
		)
	)

    ; Universal effects don't prevent rescheduling
	(:durative-action cook
		:parameters (?r)
		:duration (= ?duration 1)
		:condition (and
		    (at start (Stove ?r))
		)
		:effect (forall (?b) (when (over all (In ?b ?r))
		                           (at end (Cooked ?b))))
	)

    ; OPTIC supports classical actions
	;(:action instant-cook
	;	:parameters (?r)
	;	:condition (Stove ?r)
	;	:effect (forall (?b) (when (In ?b ?r)
	;	                           (Cooked ?b)))
	;)

	; OPTIC supports processes
    ;(:process
    ;    :parameters ()
    ;    :precondition
    ;    :effect
    ;)

    ; TODO: include any derived predicates introduce universal conditions that prevent rescheduling
    ; Only Temporal FastDownward supports derived predicates
    ;(:derived (In ?b ?r)
    ;    (exists (?p) (and (Contain ?b ?p ?r)
    ;                      (AtPose ?b ?p))))

    ;(:derived (Holding ?b)
    ;    (exists (?g) (and (Grasp ?b ?g)
    ;                      (AtGrasp ?b ?g))))
)