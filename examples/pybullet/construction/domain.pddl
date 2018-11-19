(define (domain construction)
  (:requirements :strips :equality)
  (:predicates
    (Node ?n)
    (Element ?e)
    (Printed ?e)
    (Removed ?e)
    (Traj ?t)
    (PrintAction ?n ?e ?t)
    (Collision ?t ?e)
    (CFree ?t ?e)
    (Grounded ?n)
    (Connected ?n)
    (Edge ?n1 ?e ?n2)
    (StartNode ?n ?e)
    (SecondConnection ?n ?e)
    (Supported ?e)
    (Supports ?e ?n)
    ; (Grounded ?e)
  )
  ; Most constrained -> least constrained
  ; TODO: could implement as a state constraint. At each timestep, all nodes are supported or connected
  ; For all elements, each is supported

  (:action print
    :parameters (?n ?e ?t)
    :precondition (and (PrintAction ?n ?e ?t) (Printed ?e)
                       (forall (?e2) (imply (Supports ?e2 ?n) (Printed ?e2)))
                       ; (Connected ?n)
                       ; (Supported ?n)
                       ; (SecondConnection ?n ?e)
                       ;(forall (?e2) (imply (Printed ?e2) (CFree ?t ?e2))) ; Slow to encode because of fluent
                       ;(forall (?e2) (imply (Element ?e2)
                       ;                     (or (Connected ?e2) (Removed ?e2))))
                       ;(forall (?e2) (imply (Element ?e2)
                       ;                     (or (CFree ?t ?e2) (Removed ?e2)))))
                       (forall (?e2) (imply (Collision ?t ?e2) (Removed ?e2))))
    :effect (and (Removed ?e)
                 (not (Printed ?e)))
  )

  ;(:derived (Supported ?e2) ; Single support
  ;  (and (Element ?e2) (Printed ?e2)
  ;      (or (and (Grounded ?e2))
  ;          (exists (?e1) (and (Supports ?e1 ?e2) (Supported ?e1)))))
  ;)
  ;(:derived (Supported ?n) ; All support
  ;  (and (Node ?n)
  ;       (forall (?e) (imply (Supports ?e ?n) (Printed ?e)))) ; TODO: bug in focused algorithm (preimage fact not achievable)
  ;)

  ;(:derived (Connected ?n2)
  ;  (or (Grounded ?n2)
  ;      (exists (?n1 ?e) (and (Edge ?n1 ?e ?n2)
  ;                            (Printed ?e) (Connected ?n1)))) ; Can also just do on StartNode
  ;)
  ; Either there exist another path or the connected node will not be printable

  ; Pairwise printable, exists two
  (:derived (SecondConnection ?n ?e2) ; Prevents printing when only connected through ?e2
    (and (StartNode ?n ?e2)
         (or (Grounded ?n)
             (exists (?e1) (and (StartNode ?n ?e1) (Printed ?e1) (not (= ?e1 ?e2))))) ; (Connected ?n1)
    )
  )
)