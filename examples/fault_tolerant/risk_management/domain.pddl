(define (domain risk)
    (:requirements :strips :action-costs)

    (:predicates
        ; The connectivity between nodes in the graph
        (CONNECTED ?x ?y)
        ; Nodes that can serve as traversal source
        (SOURCE ?x)
        ; Nodes that can serve as traversal target
        (TARGET ?x)
        ; Nodes that we are interested in traversing (points of interest)
        (POI ?x)
        ; The order in which not traversed POIs can be discarded
        (DISCARD_AFTER ?x ?y)

        ; Current location in the graph
        (at ?x)
        ; Whether the POI was already considered (traversed or discarded)
        (considered ?x)
        ; Whether we can now consider POIs during traversal
        (__can_explain)
        ; Whether the discard stage of the plan has started
        (__started_discard)
        ; Whether the traversal terminated
        (__goal-achieved)
        ; Whether the start operator was applied
        (__started)
        ; Whether at least one edge traversal was performed
        (__traversed)
    )

    (:functions
        (connected-cost ?x ?y) - number
        (starting-cost ?x) - number
        (discard-cost) - number
        (total-cost) - number
    )

    ;; Traverse an edge
    (:action traverse
     :parameters (?from ?to)
     :precondition (and
                        (CONNECTED ?from ?to)
                        (__can_explain)
                        (not (__started_discard))
                        (at ?from)
                   )
     :effect (and
                  (at ?to) (not (at ?from))
                  (__traversed)
                  ;(increase (total-cost) (connected-cost ?from ?to))
                  (increase (total-cost) 1)
             )
    )

    ;; goal achieving action
    (:action achieve-goal
     :parameters (?x)
     :precondition (and
                        (TARGET ?x)
                        (at ?x)
                        (not (__goal-achieved))
                        (__traversed)
                   )
     :effect (and
                  (__goal-achieved)
                  (not (at ?x))
                  (increase (total-cost) 0)
             )
    )


    ;; Choose a starting point
    (:action enter
     :parameters (?x)
     :precondition (and
                        (SOURCE ?x)
                        (not (__started))
                   )
     :effect (and
                  (__started)
                  (__can_explain)
                  (at ?x)
                  ;(increase (total-cost) (starting-cost ?x))
                  (increase (total-cost) 1)
             )
    )

    ;; mark POI as traversed
    (:action explain
     :parameters (?x)
     :precondition (and
                        (POI ?x)
                        (__can_explain)
                        (at ?x)
                        (not (considered ?x))
                        (not (__started_discard))
                   )
     :effect (and
                  (considered ?x)
                  (increase (total-cost) 1)
             )
    )

    ;; discard POI without traversing it
    (:action discard
     :parameters (?x ?y)
     :precondition (and
                        (DISCARD_AFTER ?x ?y)
                        (POI ?x)
                        (considered ?y)
                        (__goal-achieved)
                        (not (considered ?x))
                   )
     :effect (and
                  (__started_discard)
                  (not (__can_explain))
                  (considered ?x)
                  ;(increase (total-cost) (discard-cost))
                  (increase (total-cost) 1)
             )
    )
)