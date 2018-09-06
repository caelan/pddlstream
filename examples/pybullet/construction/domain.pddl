(define (domain pick-and-place)
  (:requirements :strips :equality)
  (:predicates
    (Stackable ?o ?r)
    (Sink ?r)
    (Stove ?r)

    (Grasp ?o ?g)
    (Kin ?o ?p ?g ?q ?t)
    (FreeMotion ?q1 ?t ?q2)
    (HoldingMotion ?q1 ?t ?q2 ?o ?g)
    (Supported ?o ?p ?r)
    (TrajCollision ?t ?o2 ?p2)

    (AtPose ?o ?p)
    (AtGrasp ?o ?g)
    (HandEmpty)
    (AtConf ?q)
    (CanMove)
    (Cleaned ?o)
    (Cooked ?o)

    (On ?o ?r)
    (Holding ?o)
    (UnsafeTraj ?t)
  )

  ; Negative printed for the fluent condition
  ;(:action print
  ;  :parameters (?e)
  ;  :precondition (and (Element ?e)
  ;                     (Connectable ?e) (not (Printed ?e))) ; (~Printed ?e))
  ;  :effect (and (Printed ?e)
  ;               (not (~Printed ?e)))
  ;)

  (:action print-1-2
    :parameters (?n1 ?e ?n2)
    :precondition (and (Connection ?n1 ?e ?n2)
                       (Connected ?n1) (not (Printed ?e)))
    :effect (and (Printed ?e1) (Connected ?e1))
  )
  (:action move
    :parameters (?n1 ?n2)
    :precondition (and (Node ?1)
                       (AtNode ?n1))
    :effect (and (AtNode ?n2)
                 (not (AtNode ?n1)))
  )

  ;(:derived (On ?o ?r)
  ;  (exists (?p) (and (Supported ?o ?p ?r)
  ;                    (AtPose ?o ?p)))
  ;)
)