;; Data Network Planning Domain
;; Authors: Manuel Heusner, Florian Pommerening, Alvaro Torralba
;;
;; In a given network of servers, each server can produce data by processing
;; existing data and send the data to connected servers. Each server has a disk
;; and random access memory (RAM). Data that is saved on the disk of a server
;; must be loaded into RAM of the server in order to be processed or sent to the
;; RAM of another server.
;;
;; The ability to process and distribute the data in the network is constrained
;; by
;;     - the connections between servers,
;;     - the capacity of server's RAM,
;;     - the availability of scripts on servers, and
;;     - the cost of
;;         - loading and saving data, which depends on the data size and the (disk's io performance of a) server,
;;         - sending data, which depends on the data size and (bandwidth of a) connection, and
;;         - processing data, which depends on the script and (clock rate and numbers of processors of a) server.
(define (domain data-network)
(:requirements :adl :negative-preconditions :equality :action-costs)
(:predicates
    (data ?d)
    (server ?s)
    (script ?sc)
    (numbers ?num)
    (SCRIPT-IO ?s ?in1 ?in2 ?out)
    (CONNECTED ?from ?to)
    (DATA-SIZE ?d ?n)
    (CAPACITY ?s ?n)
    (SUM ?n1 ?n2 ?sum)
    (LESS-EQUAL ?n1 ?n2)
    (saved ?d ?s)
    (cached ?d ?s)
    (usage ?s ?n)
)
(:functions
    (total-cost)
    (process-cost ?sc ?s)
    (send-cost ?from ?to ?size)
    (io-cost ?s ?size)
)
;; Release data from RAM.
(:action release
    :parameters (?d ?s ?size ?before ?after)
    :precondition
    (and
        (data ?d)
        (server ?s)
        (numbers ?size)
        (numbers ?before)
        (numbers ?after)
        (DATA-SIZE ?d ?size)
        (SUM ?after ?size ?before)
        (cached ?d ?s)
        (usage ?s ?before)
    )
    :effect
    (and
        (not (cached ?d ?s))
        (not (usage ?s ?before))
        (usage ?s ?after)
        (increase (total-cost) 0)
    )
)

;; Save data from RAM to disk.
(:action save
    :parameters (?d ?size ?s)
    :precondition
    (and
        (data ?d)
        (numbers ?size)
        (server ?s)
        (DATA-SIZE ?d ?size)
        (cached ?d ?s)
    )
    :effect
    (and
        (saved ?d ?s)
        ;(increase (total-cost) (io-cost ?s ?size))
        (increase (total-cost) 1)
    )
)

;; Load data from disk into RAM.
(:action load
    :parameters (?d ?s ?size ?limit ?before ?after)
    :precondition
    (and
        (data ?d)
        (server ?s)
        (numbers ?size)
        (numbers ?limit)
        (numbers ?before)
        (numbers ?after)
        (DATA-SIZE ?d ?size)
        (CAPACITY ?s ?limit)
        (SUM ?before ?size ?after)
        (LESS-EQUAL ?after ?limit)
        (saved ?d ?s)
        (not (cached ?d ?s))
        (usage ?s ?before)
    )
    :effect
    (and
        (cached ?d ?s)
        (not (usage ?s ?before))
        (usage ?s ?after)
        ;(increase (total-cost) (io-cost ?s ?size))
        (increase (total-cost) 1)
    )
)

;; Send data from RAM of one server to RAM of another server.
(:action send
    :parameters (?d ?from ?to ?size ?limit ?before ?after)
    :precondition
    (and
        (data ?d)
        (server ?from)
        (server ?to)
        (numbers ?size)
        (numbers ?limit)
        (numbers ?before)
        (numbers ?after)
        (CONNECTED ?from ?to)
        (DATA-SIZE ?d ?size)
        (CAPACITY ?to ?limit)
        (SUM ?before ?size ?after)
        (LESS-EQUAL ?after ?limit)
        (cached ?d ?from)
        (not (cached ?d ?to))
        (usage ?to ?before)
    )
    :effect
    (and
        (cached ?d ?to)
        (not (usage ?to ?before))
        (usage ?to ?after)
        ;(increase (total-cost) (send-cost ?from ?to ?size))
        (increase (total-cost) 1)
    )
)

;; Executes a script that processes two data items from RAM and produces another data item in RAM.
(:action process
    :parameters (?in1 ?in2 ?out ?sc ?s ?size ?limit ?before ?after)
    :precondition
    (and
        (data ?in1)
        (data ?in2)
        (data ?out)
        (script ?sc)
        (server ?s)
        (numbers ?size)
        (numbers ?limit)
        (numbers ?before)
        (numbers ?after)
        (SCRIPT-IO ?sc ?in1 ?in2 ?out)
        (DATA-SIZE ?out ?size)
        (CAPACITY ?s ?limit)
        (SUM ?before ?size ?after)
        (LESS-EQUAL ?after ?limit)
        (cached ?in1 ?s)
        (cached ?in2 ?s)
        (not (cached ?out ?s))
        (usage ?s ?before)
    )
    :effect
    (and
        (cached ?out ?s)
        (not (usage ?s ?before))
        (usage ?s ?after)
        ;(increase (total-cost) (process-cost ?sc ?s))
        (increase (total-cost) 1)
    )
)

)