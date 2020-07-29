(define (domain nightout)
(:requirements :typing :fluents :durative-actions
                 :duration-inequalities)
(:types person location machine)

(:predicates
                (finished)
		)
(:functions     (inpocket ?p - person)
                (minwithdraw ?m - machine)
                (maxwithdraw ?m - machine)
)


(:durative-action withdraw_money
:parameters (?p - person ?m - machine)
:control (?cash ?times - number)
:duration (= ?duration 2)
:condition (and (at start (>= ?cash 0))
    (at start (<= ?cash 10))
                (at start (>= (maxwithdraw ?m) 0)))
:effect (and
                (at start (decrease (maxwithdraw ?m) ?cash))
                (at end (increase (inpocket ?p) ?cash))
        ))

(:action finish
:parameters (?p - person)
:precondition (and (>= (inpocket ?p) 50) )
:effect (and (finished)) )

)
