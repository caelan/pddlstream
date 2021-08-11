(define (domain TAMP-2D)
  (:requirements :strips :equality :adl :derived-predicates :action-costs) ; Optional
  (:predicates ; Boolean functions

    ; Static type predicates
    (Body ?b) (Region ?r) (Stove ?r) (Pose ?b ?p) (Grasp ?b ?g) (Conf ?q) (Traj ?t)

    ; Static constraint predicates
    (Contain ?b ?p ?r) (Kin ?b ?q ?p ?g) (Motion ?q1 ?t ?q2) (CFree ?b1 ?p1 ?b2 ?p2)

    ; Fluent predicates
    (AtPose ?b ?p) (AtGrasp ?b ?g) (AtConf ?q) (HandEmpty) (Cooked ?b)

    ; Derived predicates
    (In ?b ?r) (Holding ?b) (UnsafePose ?b ?p)
  )

  (:functions ; Cost functions
    (Dist ?q1 ?q2)
  )

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  (:derived (In ?b ?r)        ; Derived predicate
    (exists (?p)              ; Quantified parameters
      (and (Contain ?b ?p ?r) ; Static preconditions
           (AtPose ?b ?p))))  ; Fluent preconditions

  (:derived (Holding ?b)      ; Derived predicate
    (exists (?g)              ; Quantified parameters
      (and (Grasp ?b ?g)      ; Static preconditions
           (AtGrasp ?b ?g)))) ; Fluent preconditions

  (:derived (UnsafePose ?b1 ?p1)         ; Derived predicate
    (exists (?b2 ?p2)                    ; Quantified parameters
      (and (Pose ?b1 ?p1) (Pose ?b2 ?p2) ; Static preconditions
           (not (= ?b1 ?b2))             ; Negated equality preconditions
           (not (CFree ?b1 ?p1 ?b2 ?p2)) ; Negated static preconditions
           (AtPose ?b2 ?p2))))           ; Fluent preconditions

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  (:action move                                           ; Action name
    :parameters (?q1 ?t ?q2)                              ; Action parameters
    :precondition (and (Motion ?q1 ?t ?q2)                ; Static preconditions
                       (AtConf ?q1))                      ; Fluent preconditions
    :effect (and (AtConf ?q2)                             ; Add effects
                 (not (AtConf ?q1))                       ; Delete effects
                 (increase (total-cost) (Dist ?q1 ?q2)))) ; Cost term

  (:action pick                                                ; Action name
    :parameters (?b ?p ?g ?q)                                  ; Action parameters
    :precondition (and (Kin ?b ?q ?p ?g)                       ; Static preconditions
                       (AtConf ?q) (AtPose ?b ?p) (HandEmpty)) ; Fluent preconditions
    :effect (and (AtGrasp ?b ?g)                               ; Add effects
                 (not (AtPose ?b ?p)) (not (HandEmpty))        ; Delete effects
                 (increase (total-cost) 1)))                   ; Cost term

  (:action place                                   ; Action name
    :parameters (?b ?p ?g ?q)                      ; Action parameters
    :precondition (and (Kin ?b ?q ?p ?g)           ; Static preconditions
                       (AtConf ?q) (AtGrasp ?b ?g) ; Fluent preconditions
                       (not (UnsafePose ?b ?p)))   ; Negated derived preconditions
    :effect (and (AtPose ?b ?p) (HandEmpty)        ; Add effects
                 (not (AtGrasp ?b ?g))             ; Delete effects
                 (increase (total-cost) 1)))       ; Cost term

  (:action cook                              ; Action name
    :parameters (?b ?r)                      ; Action parameters
    :precondition (and (Body ?b) (Stove ?r)  ; Static preconditions
                       (In ?b ?r))           ; Fluent preconditions
    :effect (and (Cooked ?b)                 ; Add effects
                 (increase (total-cost) 1))) ; Cost term
)
