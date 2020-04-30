(define (domain fault_tolerant)
    (:requirements :strips :equality)
    (:predicates
        ; static
        (Road ?location1 ?location2)
        (Truck ?truck)
        (Package ?package)
        (Location ?location)
        (Warehouse ?location)
        (Retailer ?location)

        ; stream
        (Open ?location1 ?location2)

        ; fluent
        (AtLocation ?truck ?location)
        (Active ?truck)
        (Carrying ?truck ?package)
    )

    (:action move
    	:parameters (?truck ?location1 ?location2)
    	:precondition
    		(and ;(Open ?location1 ?location2) ; (Active ?truck)
    		     (Truck ?truck) (Road ?location1 ?location2)
    			 (AtLocation ?truck ?location1))
    	:effect
    		(and (AtLocation ?truck ?location2)
    			 (not (AtLocation ?truck ?location1))
    			 (increase (total-cost) 1))
    )
    (:action load
    	:parameters (?truck ?package ?location)
    	:precondition
    		(and (Truck ?truck) (Package ?package) (Warehouse ?location) ; (Active ?truck)
    			 (AtLocation ?truck ?location) (AtLocation ?package ?location))
    	:effect
    		(and (Carrying ?truck ?package)
    			 (not (AtLocation ?package ?location))
    			 (increase (total-cost) 1))
    )
    (:action unload ; deliver
    	:parameters (?truck ?package ?location)
    	:precondition
    		(and (Truck ?truck) (Package ?package) (Retailer ?location) ; (Active ?truck)
    			 (AtLocation ?truck ?location) (Carrying ?truck ?package))
    	:effect
    		(and (AtLocation ?package ?location)
    			 (not (Carrying ?truck ?package))
    			 (increase (total-cost) 1))
    )

    ;(:action activate
    ;	:parameters (?truck)
    ;	:precondition
    ;		(and (Truck ?truck)
    ;		     (AllInactive))
    ;	:effect
    ;		(and (Active ?truck)
    ;		     (not (AllInactive))
    ;			 (increase (total-cost) 0))
    ;)
)

