(define (domain blocks)
  (:requirements :strips)
  (:predicates
    ; Types
    (IsBlock ?block)

    ; Controllers
    (PickController ?pickparam1 ?pickparam2)
    (PlaceController ?placeparam1 ?placeparam2)

    ; Read-off fluents
    (HasPoseX ?block ?block-posex)
    (HasPoseY ?block ?block-posey)
    (HasHeld ?block ?block-held)
    (HasClear ?block ?block-clear)

    ; AreAttrsFor predicates
    (AreAttrsForObject ?block ?block-posex ?block-posey ?block-held ?block-clear)

    ; Constraints
    (pickfromtable-constraint ?block-posex ?block-posey ?block-held ?block-clear ?pickparam1 ?pickparam2 ?next-block-posex ?next-block-posey ?next-block-held)
    (placeontable-constraint ?block-posex ?block-posey ?block-held ?block-clear ?placeparam1 ?placeparam2 ?next-block-posex ?next-block-posey ?next-block-held)
    (unstack-constraint ?block1-posex ?block1-posey ?block1-held ?block1-clear ?block2-posex ?block2-posey ?block2-held ?block2-clear ?pickparam1 ?pickparam2 ?next-block1-posex ?next-block1-posey ?next-block1-held ?next-block2-clear)
    (stack-constraint ?block1-posex ?block1-posey ?block1-held ?block1-clear ?block2-posex ?block2-posey ?block2-held ?block2-clear ?placeparam1 ?placeparam2 ?next-block1-posex ?next-block1-posey ?next-block1-held ?next-block2-clear)

    ; Goal fluents
    (On ?b1 ?b2)
    (handempty)
    (holding ?b)
    (ontable ?b)
    (clear ?b)
  )

  (:action pickfromtable
    :parameters (?block ?block-posex ?block-posey ?block-held ?block-clear ?pickparam1 ?pickparam2 ?next-block-posex ?next-block-posey ?next-block-held)
    :precondition (and
        ; Object types
        (IsBlock ?block)
        (handempty)
        (ontable ?block)
        (clear ?block)

        ; Controller
        (PickController ?pickparam1 ?pickparam2)

        ; Read-off fluents
        (HasPoseX ?block ?block-posex)
        (HasPoseY ?block ?block-posey)
        (HasHeld ?block ?block-held)
        (HasClear ?block ?block-clear)

        ; Constraint
        (pickfromtable-constraint ?block-posex ?block-posey ?block-held ?block-clear ?pickparam1 ?pickparam2 ?next-block-posex ?next-block-posey ?next-block-held)
    )
    :effect (and
        ; Read-off fluents
        (not (HasPoseX ?block ?block-posex))
        (HasPoseX ?block ?next-block-posex)
        (not (HasPoseY ?block ?block-posey))
        (HasPoseY ?block ?next-block-posey)
        (not (HasHeld ?block ?block-held))
        (HasHeld ?block ?next-block-held)

        ; Goal fluents
        (not (handempty))
        (holding ?block)
        (not (ontable ?block))
        (not (clear ?block))
    )
  )

  (:action placeontable
    :parameters (?block ?block-posex ?block-posey ?block-held ?block-clear ?placeparam1 ?placeparam2 ?next-block-posex ?next-block-posey ?next-block-held)
    :precondition (and
        ; Object types
        (IsBlock ?block)
        (holding ?block)

        ; Controller
        (PlaceController ?placeparam1 ?placeparam2)

        ; Read-off fluents
        (HasPoseX ?block ?block-posex)
        (HasPoseY ?block ?block-posey)
        (HasHeld ?block ?block-held)
        (HasClear ?block ?block-clear)

        ; Constraint
        (placeontable-constraint ?block-posex ?block-posey ?block-held ?block-clear ?placeparam1 ?placeparam2 ?next-block-posex ?next-block-posey ?next-block-held)
    )
    :effect (and
        ; Read-off fluents
        (not (HasPoseX ?block ?block-posex))
        (HasPoseX ?block ?next-block-posex)
        (not (HasPoseY ?block ?block-posey))
        (HasPoseY ?block ?next-block-posey)
        (not (HasHeld ?block ?block-held))
        (HasHeld ?block ?next-block-held)

        ; Goal fluents
        (handempty)
        (not (holding ?block))
        (ontable ?block)
        (clear ?block)
    )
  )

  (:action unstack
    :parameters (?block1 ?block2 ?block1-posex ?block1-posey ?block1-held ?block1-clear ?block2-posex ?block2-posey ?block2-held ?block2-clear ?pickparam1 ?pickparam2 ?next-block1-posex ?next-block1-posey ?next-block1-held ?next-block2-clear)
    :precondition (and
        ; Object types
        (IsBlock ?block1)
        (IsBlock ?block2)
        (handempty)
        (On ?block1 ?block2)
        (clear ?block1)

        ; Controller
        (PickController ?pickparam1 ?pickparam2)

        ; Read-off fluents
        (HasPoseX ?block1 ?block1-posex)
        (HasPoseY ?block1 ?block1-posey)
        (HasHeld ?block1 ?block1-held)
        (HasClear ?block1 ?block1-clear)
        (HasPoseX ?block2 ?block2-posex)
        (HasPoseY ?block2 ?block2-posey)
        (HasHeld ?block2 ?block2-held)
        (HasClear ?block2 ?block2-clear)

        ; Constraint
        (unstack-constraint ?block1-posex ?block1-posey ?block1-held ?block1-clear ?block2-posex ?block2-posey ?block2-held ?block2-clear ?pickparam1 ?pickparam2 ?next-block1-posex ?next-block1-posey ?next-block1-held ?next-block2-clear)

    )
    :effect (and
        ; Read-off fluents
        (not (HasPoseX ?block1 ?block1-posex))
        (HasPoseX ?block1 ?next-block1-posex)
        (not (HasPoseY ?block1 ?block1-posey))
        (HasPoseY ?block1 ?next-block1-posey)
        (not (HasHeld ?block1 ?block1-held))
        (HasHeld ?block1 ?next-block1-held)
        (not (HasClear ?block2 ?block2-clear))
        (HasClear ?block2 ?next-block2-clear)

        ; Goal fluents
        (not (On ?block1 ?block2))
        (not (handempty))
        (holding ?block1)
        (not (clear ?block1))
        (clear ?block2)
    )
  )

  (:action stack
    :parameters (?block1 ?block2 ?block1-posex ?block1-posey ?block1-held ?block1-clear ?block2-posex ?block2-posey ?block2-held ?block2-clear ?placeparam1 ?placeparam2 ?next-block1-posex ?next-block1-posey ?next-block1-held ?next-block2-clear)
    :precondition (and
        ; Object types
        (IsBlock ?block1)
        (IsBlock ?block2)
        (holding ?block1)
        (clear ?block2)

        ; Controller
        (PlaceController ?placeparam1 ?placeparam2)

        ; Read-off fluents
        (HasPoseX ?block1 ?block1-posex)
        (HasPoseY ?block1 ?block1-posey)
        (HasHeld ?block1 ?block1-held)
        (HasClear ?block1 ?block1-clear)
        (HasPoseX ?block2 ?block2-posex)
        (HasPoseY ?block2 ?block2-posey)
        (HasHeld ?block2 ?block2-held)
        (HasClear ?block2 ?block2-clear)

        ; Constraint
        (stack-constraint ?block1-posex ?block1-posey ?block1-held ?block1-clear ?block2-posex ?block2-posey ?block2-held ?block2-clear ?placeparam1 ?placeparam2 ?next-block1-posex ?next-block1-posey ?next-block1-held ?next-block2-clear)
    )
    :effect (and
        ; Read-off fluents
        (not (HasPoseX ?block1 ?block1-posex))
        (HasPoseX ?block1 ?next-block1-posex)
        (not (HasPoseY ?block1 ?block1-posey))
        (HasPoseY ?block1 ?next-block1-posey)
        (not (HasHeld ?block1 ?block1-held))
        (HasHeld ?block1 ?next-block1-held)
        (not (HasClear ?block2 ?block2-clear))
        (HasClear ?block2 ?next-block2-clear)

        ; Goal fluents
        (On ?block1 ?block2)
        (handempty)
        (not (holding ?block1))
        (not (clear ?block2))
        (clear ?block1)
    )
  )

)