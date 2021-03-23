(define (domain nightout)
    (:requirements :typing)
    ;(:types person location machine)
    (:constants @amount) ; @min @max)
    (:predicates
        (person ?p)
        (machine ?m)
        (pcash ?c)
        (wcash ?c)
        (mcash ?c)

        (ge ?c1 ?c2)
        (sum ?c1 ?c2 ?c3)
        (withdraw ?wcash ?pcash1 ?pcash2 ?mcash1 ?mcash2)

        (finished)
        (inpocket ?p ?c)
        (maxwithdraw ?m ?c)
    )
    (:functions
        ;(minwithdraw ?m - machine)
        (withdrawcost ?wcash)
    )

    ; TODO: convert typing using diverse code
    (:action withdraw_money
     :parameters (?p ?m ?wcash ?pcash1 ?pcash2 ?mcash1 ?mcash2)
     :precondition (and (person ?p) (machine ?m)
                        ; (ge ?wcash @min) (ge @max ?wcash) (ge ?mcash1 ?wcash)
                        ;(sum ?pcash1 ?wcash ?pcash2) (sum ?mcash2 ?wcash ?mcash1) ; TODO: separate
                        (withdraw ?wcash ?pcash1 ?pcash2 ?mcash1 ?mcash2)
                        (inpocket ?p ?pcash1) (maxwithdraw ?m ?mcash1))
     :effect (and (inpocket ?p ?pcash2) (maxwithdraw ?m ?mcash2)
                  (not (inpocket ?p ?pcash1)) (not (maxwithdraw ?m ?mcash1))
                  ;(increase (total-cost) 2)))
                  (increase (total-cost) (withdrawcost ?wcash))))

    (:action finish
     ;:parameters (?p ?pcash) ; TODO: strange bug due to constants
     ;:precondition (and (person ?p) (ge ?pcash @amount) ; (pcash ?wcash)
     :parameters (?p ?pcash ?tcash)
     :precondition (and (person ?p) (ge ?pcash ?tcash)
                        (inpocket ?p ?pcash))
     :effect (finished))
)
; https://github.com/Emresav/ECAI16Domains/blob/master/cashpoint/domain0.pddl
