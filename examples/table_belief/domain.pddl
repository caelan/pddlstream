(define (domain latent-table)
  (:requirements :strips :equality)
  (:predicates
    (On ?i ?s)
    (Holding ?a ?b)
    (HandEmpty ?a)

    (Arm ?a)
    (Unknown ?s)
    (Localized ?s)
    (Registered ?s)

    (Nearby ?s)
    (Graspable ?i)
    (Stackable ?i ?s)
    (CanMove)
    (Class ?i ?c)
    (HoldingClass ?c)

    (FiniteScanCost ?s ?i)
  )
  (:functions
    (Distance ?q1 ?q2)
    (ScanCost ?s ?i)
    (RegisterCost)
    (PickCost)
    (PlaceCost)
  )
  ;(:action move
  ;  :parameters (?s)
  ;  :precondition (Fixed ?s)
  ;  :effect (Nearby ?s)
  ;)

  (:action scan
    :parameters (?s ?i)
    :precondition (and (FiniteScanCost ?s ?i)
                   (Localized ?s) (Unknown ?i))
    :effect (and (Localized ?i) (On ?i ?s)
                 (not (Unknown ?i))
                 (increase (total-cost) (ScanCost ?s ?i)))
  )
  (:action register
    :parameters (?i) ; Can rooms and tables be registered?
    :precondition (and (Graspable ?i) (Localized ?i))
    :effect (and (Registered ?i)
                 (increase (total-cost) (RegisterCost)))
  )

  (:action pick
    :parameters (?a ?i ?s)
    :precondition (and (Arm ?a) (Stackable ?i ?s) (Graspable ?i)
                       (On ?i ?s) (HandEmpty ?a) (Registered ?i) (Localized ?s)) ; (Nearby ?s)
    :effect (and (Holding ?a ?i) ; (CanMove)
                 (not (On ?i ?s)) (not (HandEmpty ?a))
                 (not (Localized ?i)) (not (Registered ?i))
                 (increase (total-cost) (PickCost)))
  )
  (:action place
    :parameters (?a ?i ?s)
    :precondition (and (Arm ?a) (Stackable ?i ?s) (Graspable ?i)
                       (Holding ?a ?i) (Localized ?s)) ; (Nearby ?s)
    :effect (and (On ?i ?s) (HandEmpty ?a) (Localized ?i) ; (CanMove)
                 (not (Holding ?a ?i))
                 (increase (total-cost) (PlaceCost)))
  )

  (:derived (HoldingClass ?c)
    (exists (?a ?i) (and (Arm ?a) (Class ?i ?c)
                      (Holding ?a ?i)))
  )
)