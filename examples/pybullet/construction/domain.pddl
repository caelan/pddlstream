(define (domain construction)
  (:requirements :strips :equality)
  (:predicates
    (Node ?n)
    (Connected ?n)
    (Element ?e)
    (Printed ?e)
    (Connection ?n1 ?e ?t ?n2)
    (AtNode ?n)
    (Traj ?t)
    (CFree ?t ?e)
    (Unsafe ?t)
  )

  ; Negative printed for the fluent condition
  ;(:action print
  ;  :parameters (?e)
  ;  :precondition (and (Element ?e)
  ;                     (Connectable ?e) (not (Printed ?e))) ; (~Printed ?e))
  ;  :effect (and (Printed ?e)
  ;               (not (~Printed ?e)))
  ;)

  (:action print
    :parameters (?n1 ?e ?t ?n2)
    :precondition (and (Connection ?n1 ?e ?t ?n2)
                       (Connected ?n1) (not (Printed ?e)) (not (Unsafe ?t)))
    :effect (and (Connected ?n2) (Printed ?e) (AtNode ?n2)
                 (forall (?t2)
                    (when (and (Traj ?t2) (not (CFree ?t2 ?e)))
                          (Unsafe ?t2)))
                 (not (AtNode ?n1)))
    ; Conditional effect here or derived predicate?
  )

  ;(:action move
  ;  :parameters (?n1 ?n2)
  ;  :precondition (and (Node ?n1) (Node ?n2)
  ;                     (AtNode ?n1))
  ;  :effect (and (AtNode ?n2) ; (CanMove)
  ;               (not (AtNode ?n1))
  ;               (increase (total-cost) (Distance ?n1 ?n2)))
  ;)

  ;(:derived (On ?o ?r)
  ;  (exists (?p) (and (Supported ?o ?p ?r)
  ;                    (AtPose ?o ?p)))
  ;)
)