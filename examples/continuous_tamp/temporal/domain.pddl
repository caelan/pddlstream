(define (domain temporal-tamp)
    (:requirements :equality :durative-actions :numeric-fluents :derived-predicates :conditional-effects) ; :typing :action-costs
	; (:types block surface)
    (:constants a b grey red)
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
        (AtGrasp ?r ?b ?g)
        (AtConf ?r ?q)
        (HandEmpty ?r)
        (CanMove ?r)

        (Safe)
        (Stove ?s)
        (Cooked ?b)
        (SafePose ?b ?p ?b2)
				(Goal)

        ; Derived predicates
        (On ?b ?s) ; TFLAP redeclared predicate
        (Holding ?r ?b)
        (UnsafePose ?b ?p)
        (UnsafeTraj ?t)
    )
    (:functions
				(Duration ?t)
        (Dist ?q1 ?q2)
        (total-cost)
        ; (Fuel)
    )

    ; over all is an open set on time
    ; https://www.jair.org/index.php/jair/article/view/10352/24759
    ; 8.1 Durative Actions with Conditional Effects
    ; TODO: can always enumerate finite types

	(:durative-action move
		:parameters (?r ?q1 ?t ?q2)
		:duration (= ?duration (Duration ?t))
		:condition (and
			(at start (and (Robot ?r) (Motion ?q1 ?t ?q2)))
			(at start (AtConf ?r ?q1))
			;(over all (Safe))
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
			;(over all (Safe))
		)
		:effect (and
			(at start (not (AtPose ?b ?p)))
			(at start (not (HandEmpty ?r)))
			(at end (AtGrasp ?r ?b ?g))
			(forall (?s) (when (at start (Contain ?b ?p ?s)) ; TODO: maybe typing info helps here
												 (at end (not (On ?b ?s)))))
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
			(over all (not (UnsafePose ?b ?p)))
			(at end (not (UnsafePose ?b ?p)))

            ;(or (= ?b a)
            ;    ;(and (over all (SafePose ?b ?p a))
            ;    ;      (at end (SafePose ?b ?p a))
            ;    ;)
            ;)
            ;(or (= ?b b)
            ;    ;(and (over all (SafePose ?b ?p b))
            ;    ;      (at end (SafePose ?b ?p b))
            ;    ;)
            ;)
			;(over all (Safe))
		)
		:effect (and
		    (at start (not (AtGrasp ?r ?b ?g)))
			(at end (AtPose ?b ?p))
			(at end (HandEmpty ?r))
			;(forall (?b2 ?p2) (when (and (at start (not (CFree ?b ?p ?b2 ?p2))) (at end (AtPose ?b2 ?p2)))
			;                        (at end (not (Safe)))))
			;(when (at start (Contain ?b ?p red))
			;      (at end (On ?b red)))
			(forall (?s) (when (at start (Contain ?b ?p ?s))
			                   (at end (On ?b ?s))))
		)
	)
	; This is the rule of no moving targets : no concurrent actions can affect the parts of the state relevant to the precondition
    ; tests of other actions in the set, regardless of whether those effects might be harmful or not.

    ; Universal effects don't prevent rescheduling
	(:durative-action cook
		:parameters (?s)
		:duration (= ?duration 1)
		:condition (and
		    (at start (Stove ?s))
		    ;(over all (Safe))
		)
		:effect (and
			; TODO: rescheduling does not work well with some conditional effects
			; TODO: need at end otherwise it fails
			;(when (and (at start (On a ?s)) (over all (On a ?s)) (at end (On a ?s)))
			;    (at end (Cooked a)))
			; TODO: VAL doesn't detect that the following isn't a solution

		    (forall (?b) (when (and (at start (On ?b ?s)) (over all (On ?b ?s)) (at end (On ?b ?s)))
		        (at end (Cooked ?b)))) ; This doesn't seem to work
	    )
	)

	; TODO: include any derived predicates introduce universal conditions that prevent rescheduling
    ; Only Temporal FastDownward supports derived predicates

	;(:durative-action achieve-goal
	;	:parameters ()
	;	:duration (= ?duration 0)
	;	:condition (and
	;		(at start (On a red))
	;		(at start (On b red))
	;	)
	;	:effect (and
	;		(at end (Goal))
	;	)
	;)

    ;(:derived (On ?b ?s)
    ;    (exists (?p) (and (Contain ?b ?p ?s)
    ;                      (AtPose ?b ?p))))

    ;(:derived (Holding ?b)
    ;    (exists (?g) (and (Grasp ?b ?g)
    ;                      (AtGrasp ?b ?g))))

	;(:derived (SafePose ?b ?p ?b2)
	;    (and (Pose ?b ?p) (Block ?b2) (or
	;        (Holding ?b2)
    ;        (exists (?p2) (and (CFree ?b ?p ?b2 ?p2)
    ;                           (AtPose ?b2 ?p2))))))

	(:derived (UnsafePose ?b ?p)
        (exists (?b2 ?p2) (and (Pose ?b ?p) (Pose ?b2 ?p2)
                               (not (CFree ?b ?p ?b2 ?p2))
                               (AtPose ?b2 ?p2))))

    ; These are executed in parallel. Action to turn on stove and action to cook
	;(:durative-action cook
	;	:parameters (?s ?b)
	;	:duration (= ?duration 1)
	;	:condition (and
	;	    (at start (Stove ?s))
	;	    (at start (Block ?b))
	;	    (over all (On ?b ?s))
	;	)
	;	:effect (at end (Cooked ?b))
	;)

    ; OPTIC supports classical actions
	;(:action instant-cook
	;	:parameters (?s)
	;	:condition (Stove ?s)
	;	:effect (forall (?b) (when (On ?b ?s)
	;	                           (Cooked ?b)))
	;)

	; OPTIC supports processes
    ;(:process
    ;    :parameters ()
    ;    :precondition
    ;    :effect
    ;)
)