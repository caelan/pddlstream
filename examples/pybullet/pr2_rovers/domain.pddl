(define (domain rovers)
  (:requirements :strips :equality)
  ; (:constants)
  (:predicates
    (Rover ?v)
    (Objective ?o)
    (Mode ?m)
    (Rock ?r)
    (Soil ?s)
    (Store ?s)
    (Lander ?l)
    (Ray ?y)
    (BConf ?bq)
    (HConf ?hq)

    (BMotion ?bq1 ?bt ?bq2)
    (HMotion ?hq1 ?ht ?hq2)
    (ImageVisible ?bq ?hq ?y ?o)
    (ComVisible ?bq ?hq ?y ?l)
    (Above ?bq ?r)

    (AtBConf ?v ?bq)
    (AtHConf ?v ?hq)

    (OnBoard ?c ?v)
    (Supports ?c ?m)
    (Calibrated ?c ?v)
    (HaveImage ?v ?o ?m)
    (ReceivedImage ?o ?m)
    (ReceivedAnalysis ?r)
    (Analyzed ?v ?r)
    (Free ?v ?s)
    (Full ?v ?s)

    (Blocked ?y)
  )
  ; (:functions)

  (:action move_base
    :parameters (?v ?bq1 ?bt ?bq2)
    :precondition (and (Rover ?v) (BMotion ?bq1 ?bt ?bq2)
                       (AtBConf ?v ?bq1))
    :effect (and (AtBConf ?v ?bq2)
                 (not (AtBConf ?v ?bq1)))
  )
  (:action move_head
    :parameters (?v ?hq1 ?ht ?hq2)
    :precondition (and (Rover ?v) (HMotion ?hq1 ?ht ?hq2)
                       (AtHConf ?v ?hq1))
    :effect (and (AtHConf ?v ?hq2)
                 (not (AtHConf ?v ?hq1)))
  )
  (:action take_image
    :parameters (?v ?bq ?hq ?y ?o ?c ?m)
    :precondition (and (Rover ?v) (ImageVisible ?bq ?hq ?y ?o) (OnBoard ?c ?v) (Supports ?c ?m)
                       (AtBConf ?v ?bq) (AtHConf ?v ?hq) (Calibrated ?c ?v)
                       ; (not (Blocked ?y))
                  )
    :effect (and (HaveImage ?v ?o ?m)
                 (not (Calibrated ?c ?v)))
  )
  (:action calibrate
    :parameters (?v ?bq ?hq ?y ?o ?c)
    :precondition (and (Rover ?v) (ImageVisible ?bq ?hq ?y ?o) (OnBoard ?c ?v)
                       (AtBConf ?v ?bq) (AtHConf ?v ?hq)
                       ; (not (Blocked ?y))
                  )
    :effect (Calibrated ?c ?v)
  )
  (:action send_image
    :parameters (?v ?bq ?hq ?y ?l ?o ?m)
    :precondition (and (Rover ?v) (ComVisible ?bq ?hq ?y ?l) (Objective ?o) (Mode ?m)
                       (HaveImage ?v ?o ?m) (AtBConf ?v ?bq) (AtHConf ?v ?hq)
                       ; (not (Blocked ?y))
                  )
    :effect (ReceivedImage ?o ?m)
  )

  (:action sample_rock
    :parameters (?v ?bq ?r ?s)
    :precondition (and (Rover ?v) (Above ?bq ?r) (Store ?s)
                       (AtBConf ?v ?bq) (Free ?v ?s))
    :effect (and (Full ?v ?s) (Analyzed ?v ?r)
                 (not (Free ?v ?s)))
  )
  (:action send_analysis
    :parameters (?v ?bq ?hq ?y ?l ?r)
    :precondition (and (Rover ?v) (ComVisible ?bq ?hq ?y ?l) (Rock ?r)
                       (Analyzed ?v ?r) (AtBConf ?v ?bq) (AtHConf ?v ?hq)
                       ; (not (Blocked ?y))
                  )
    :effect (ReceivedAnalysis ?r)
  )
  (:action drop_rock
    :parameters (?v ?s)
    :precondition (and (Rover ?v) (Store ?s)
                       (Full ?v ?s))
    :effect (and (Free ?v ?s)
                 (not (Full ?v ?s)))
  )

  ;(:action recharge # TODO: sample states accessible by sun
  ;  :parameters (?v ?bq)
  ;  :precondition (and (InSun ?bq)
  ;                     (AtBConf ?v ?bq)
  ;                )
  ;  :effect (Charged ?v)
  ;)

  ;(:derived (Blocked ?y)
  ;  (exists (?v ?bq) (and (Ray ?y) (Rover ?v) (BConf ?bq)
  ;                         (not (CFreeRayConf ?y ?v ?bq))
  ;                         (AtBConf ?v ?bq)))
  ;)

)