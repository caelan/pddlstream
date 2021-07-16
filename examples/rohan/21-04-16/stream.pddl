(define (stream blocks)

  (:stream pickfromtable-constraint-sampler
    :inputs (?block ?block-posex ?block-posey ?block-held ?block-clear)
    :domain (and (IsBlock ?block)
                 (AreAttrsForObject ?block ?block-posex ?block-posey ?block-held ?block-clear))
    :outputs (?pickparam1 ?pickparam2 ?next-block-posex ?next-block-posey ?next-block-held)
    :certified (and (AreAttrsForObject ?block ?next-block-posex ?next-block-posey ?next-block-held ?block-clear)
                    (PickController ?pickparam1 ?pickparam2)
                    (pickfromtable-constraint ?block-posex ?block-posey ?block-held ?block-clear ?pickparam1 ?pickparam2 ?next-block-posex ?next-block-posey ?next-block-held))
  )

  (:stream placeontable-constraint-sampler
    :inputs (?block ?block-posex ?block-posey ?block-held ?block-clear)
    :domain (and (IsBlock ?block)
                 (AreAttrsForObject ?block ?block-posex ?block-posey ?block-held ?block-clear))
    :outputs (?placeparam1 ?placeparam2 ?next-block-posex ?next-block-posey ?next-block-held)
    :certified (and (AreAttrsForObject ?block ?next-block-posex ?next-block-posey ?next-block-held ?block-clear)
                    (PlaceController ?placeparam1 ?placeparam2)
                    (placeontable-constraint ?block-posex ?block-posey ?block-held ?block-clear ?placeparam1 ?placeparam2 ?next-block-posex ?next-block-posey ?next-block-held))
  )

  (:stream unstack-constraint-sampler
    :inputs (?block1 ?block2 ?block1-posex ?block1-posey ?block1-held ?block1-clear ?block2-posex ?block2-posey ?block2-held ?block2-clear)
    :domain (and (IsBlock ?block1)
                 (AreAttrsForObject ?block1 ?block1-posex ?block1-posey ?block1-held ?block1-clear)
                 (IsBlock ?block2)
                 (AreAttrsForObject ?block2 ?block2-posex ?block2-posey ?block2-held ?block2-clear))
    :outputs (?pickparam1 ?pickparam2 ?next-block1-posex ?next-block1-posey ?next-block1-held ?next-block2-clear)
    :certified (and (AreAttrsForObject ?block1 ?next-block1-posex ?next-block1-posey ?next-block1-held ?block1-clear)
                    (AreAttrsForObject ?block2 ?block2-posex ?block2-posey ?block2-held ?next-block2-clear)
                    (PickController ?pickparam1 ?pickparam2)
                    (unstack-constraint ?block1-posex ?block1-posey ?block1-held ?block1-clear ?block2-posex ?block2-posey ?block2-held ?block2-clear ?pickparam1 ?pickparam2 ?next-block1-posex ?next-block1-posey ?next-block1-held ?next-block2-clear))
  )

  (:stream stack-constraint-sampler
    :inputs (?block1 ?block2 ?block1-posex ?block1-posey ?block1-held ?block1-clear ?block2-posex ?block2-posey ?block2-held ?block2-clear)
    :domain (and (IsBlock ?block1)
                 (AreAttrsForObject ?block1 ?block1-posex ?block1-posey ?block1-held ?block1-clear)
                 (IsBlock ?block2)
                 (AreAttrsForObject ?block2 ?block2-posex ?block2-posey ?block2-held ?block2-clear))
    :outputs (?placeparam1 ?placeparam2 ?next-block1-posex ?next-block1-posey ?next-block1-held ?next-block2-clear)
    :certified (and (AreAttrsForObject ?block1 ?next-block1-posex ?next-block1-posey ?next-block1-held ?block1-clear)
                    (AreAttrsForObject ?block2 ?block2-posex ?block2-posey ?block2-held ?next-block2-clear)
                    (PlaceController ?placeparam1 ?placeparam2)
                    (stack-constraint ?block1-posex ?block1-posey ?block1-held ?block1-clear ?block2-posex ?block2-posey ?block2-held ?block2-clear ?placeparam1 ?placeparam2 ?next-block1-posex ?next-block1-posey ?next-block1-held ?next-block2-clear))
  )

)