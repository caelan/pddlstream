(define (stream TAMP-2D)

  ; Generator streams

  (:stream s-region                      ; Stream name
    :inputs (?b ?r)                      ; Input parameters
    :domain (and (Body ?b) (Region ?r))  ; Input type constraints
    :outputs (?p)                        ; Output parameters
    :certified (and (Pose ?b ?p)         ; Output type properties
                    (Contain ?b ?p ?r))) ; Output constraint properties

  (:stream s-grasp            ; Stream name
    :inputs (?b)              ; Input parameters
    :domain (Body ?b)         ; Input type constraints
    :outputs (?g)             ; Output parameters
    :certified (Grasp ?b ?g)) ; Output type properties

  (:stream s-ik                              ; Stream name
    :inputs (?b ?p ?g)                       ; Input parameters
    :domain (and (Pose ?b ?p) (Grasp ?b ?g)) ; Input type constraints
    :outputs (?q)                            ; Output parameters
    :certified (and (Conf ?q)                ; Output type properties
                    (Kin ?b ?q ?p ?g)))      ; Output constraint properties

  (:stream s-motion                       ; Stream name
    :inputs (?q1 ?q2)                     ; Input parameters
    :domain (and (Conf ?q1) (Conf ?q2))   ; Input type constraints
    :outputs (?t)                         ; Output parameters
    :certified (and (Traj ?t)             ; Output type properties
                    (Motion ?q1 ?t ?q2))) ; Output constraint properties

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  ; Test streams

  (:stream t-cfree                              ; Stream name
    :inputs (?b1 ?p1 ?b2 ?p2)                   ; Input parameters
    :domain (and (Pose ?b1 ?p1) (Pose ?b2 ?p2)) ; Input type constraints
    :certified (CFree ?b1 ?p1 ?b2 ?p2))         ; Output constraint properties

  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  ; External functions

  (:function (Dist ?q1 ?q2)      ; External function
    (and (Conf ?q1) (Conf ?q2))) ; Input type constraints
)

  ;(:stream t-region                        ; Stream name
  ;  :inputs (?b ?p ?r)                     ; Input parameters
  ;  :domain (and (Pose ?b ?p) (Region ?r)) ; Input type constraints
  ;  :certified (Contain ?b ?p ?r))         ; Output constraint properties
