(define (domain pr2-tamp)
  (:requirements :strips :equality)
  (:predicates
    (Arm ?a)
    (Graspable ?o)
    (Stackable ?o ?r)
    (Sink ?r)
    (Stove ?r)

    (Pose ?o ?p)
    (Grasp ?o ?g)
    (Kin ?a ?o ?p ?g ?q ?t)
    (BaseMotion ?q1 ?t ?q2)
    (Supported ?o ?p ?r)
    (Vis ?o ?p ?bq ?hq)

    (CanMove) ; Do I still want this?
    (AtPose ?o ?p)
    (AtGrasp ?a ?o ?g)
    (HandEmpty ?a)
    (AtBConf ?q) ; Pass in part name for simplicity
    (AtAConf ?a ?q)
    ; One conf for all of this?

    (On ?o ?r)
    (Holding ?a ?o)

    (Unknown ?o)
    (Scanned ?o)
    (Localized ?o)
    (Registered ?o)
  )

  (:action move_base
    :parameters (?q1 ?q2 ?t)
    :precondition (and (BaseMotion ?q1 ?t ?q2)
                       (AtBConf ?q1))
    :effect (and (AtBConf ?q2)
                 (not (AtBConf ?q1)))
                 ; (forall (?o) (not (Registered ?o))))
                 ; (forall (?o) (when (Graspable ?o) (not (Registered ?o)))))
  )
  (:action pick
    :parameters (?a ?o ?p ?g ?q ?t)
    :precondition (and (Kin ?a ?o ?p ?g ?q ?t)
                       (Registered ?o) (AtPose ?o ?p) (HandEmpty ?a) (AtBConf ?q))
    :effect (and (AtGrasp ?a ?o ?g)
                 (not (AtPose ?o ?p)) (not (HandEmpty ?a)))
  )
  (:action place
    :parameters (?a ?o ?p ?g ?q ?t)
    :precondition (and (Kin ?a ?o ?p ?g ?q ?t)
                       (Localized ?o) (AtGrasp ?a ?o ?g) (AtBConf ?q))
    :effect (and (AtPose ?o ?p) (HandEmpty ?a)
                 (not (AtGrasp ?a ?o ?g)))
  )

  (:action scan
    :parameters (?o ?p ?bq ?hq)
    :precondition (and (Vis ?o ?p ?bq ?hq)
                       (AtPose ?o ?p) (AtBConf ?bq) (Localized ?o))
    :effect (and (Scanned ?o))
  )
  (:action localize
    :parameters (?r ?p1 ?o ?p2)
    :precondition (and (Stackable ?o ?r) (Pose ?r ?p1) (Pose ?o ?p2) ; (FiniteScanCost ?r ?o)
                   (AtPose ?o ?p2) (Scanned ?r) (Unknown ?o))
    :effect (and (Localized ?o) (Supported ?o ?p2 ?r)
                 (not (Unknown ?o)))
                 ;(increase (total-cost) (ScanCost ?r ?o)))
  )
  (:action register
    :parameters (?o ?p ?bq ?hq)
    :precondition (and (Vis ?o ?p ?bq ?hq)
                       (AtPose ?o ?p) (AtBConf ?bq) (Localized ?o))
    :effect (and (Registered ?o))
                 ;(increase (total-cost) (RegisterCost)))
  )

  (:derived (On ?o ?r)
    (exists (?p) (and (Supported ?o ?p ?r)
                      (AtPose ?o ?p)))
  )
  (:derived (Holding ?a ?o)
    (exists (?g) (and (Arm ?a) (Grasp ?o ?g)
                      (AtGrasp ?a ?o ?g)))
  )
)