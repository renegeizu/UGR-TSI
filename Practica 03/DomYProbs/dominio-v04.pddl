(define (domain zeno-travel)

(:requirements :typing :fluents :derived-predicates :negative-preconditions :universal-preconditions :disjuntive-preconditions :conditional-effects :htn-expansion :durative-actions :metatags)

(:types aircraft person city - object)

(:constants slow fast - object)

(:predicates (at ?x - (either person aircraft) ?c - city) 
             (in ?p - person ?a - aircraft)
			 (different ?x ?y)
			 (igual ?x ?y)
			 (hay-fuel-slow ?a ?c1 ?c2)
			 (hay-fuel-fast ?a ?c1 ?c2)
             (available-fuel-slow ?a ?c1 ?c2)
             (available-fuel-fast ?a ?c1 ?c2)
			 (destination ?p - person ?c - city)
			 (available-duration-fast ?a - aircraft ?c1 - city ?c2 - city)
			 (available-duration-slow ?a - aircraft ?c1 - city ?c2 - city)
)

(:functions (fuel ?a - aircraft) 
            (distance ?c1 - city ?c2 - city)
			(slow-speed ?a - aircraft)
            (fast-speed ?a - aircraft)
            (slow-burn ?a - aircraft)
            (fast-burn ?a - aircraft)
            (capacity ?a - aircraft)
            (refuel-rate ?a - aircraft)
            (total-fuel-used ?a - aircraft)
            (boarding-time)
            (debarking-time)
			(fuel-limit ?a - aircraft)
			(num-passenger ?a - aircraft)
			(capacity-passenger ?a - aircraft)
			(duration ?a - aircraft)
			(max-duration ?a - aircraft)
)

(:derived (igual ?x ?x) ())

(:derived (different ?x ?y) (not (igual ?x ?y)))

(:derived (hay-fuel-slow ?a - aircraft ?c1 - city ?c2 - city) (>= (fuel ?a) (* (distance ?c1 ?c2) (slow-burn ?a))))

(:derived (hay-fuel-fast ?a - aircraft ?c1 - city ?c2 - city) (>= (fuel ?a) (* (distance ?c1 ?c2) (fast-burn ?a))))

(:derived (available-fuel-slow ?a - aircraft ?c1 - city ?c2 - city) (< (+ (total-fuel-used ?a) (* (distance ?c1 ?c2) (slow-burn ?a))) (fuel-limit ?a)))

(:derived (available-fuel-fast ?a - aircraft ?c1 - city ?c2 - city) (< (+ (total-fuel-used ?a) (* (distance ?c1 ?c2) (fast-burn ?a))) (fuel-limit ?a)))

(:derived (distance ?c2 - city ?c1 - city) (distance ?c1 ?c2))

(:derived (available-duration-fast ?a - aircraft ?c1 - city ?c2 - city) (<= (+ (duration ?a) (/ (distance ?c1 ?c2) (fast-speed ?a))) (max-duration ?a)))

(:derived (available-duration-slow ?a - aircraft ?c1 - city ?c2 - city) (<= (+ (duration ?a) (/ (distance ?c1 ?c2) (slow-speed ?a))) (max-duration ?a)))

(:task transport-person
 :parameters (?p - person ?c - city)
			 (:method Case1 :precondition (and (at ?p ?c) (destination ?p ?c))
							:tasks ())
			 (:method Case2 :precondition (and (at ?p - person ?c1 - city) (at ?a - aircraft ?c1 - city) (not (destination ?p - person ?c1 - city)))
							:tasks ((board-all ?c) (mover-avion ?a ?c1 ?c) (transport-person ?p ?c)))
			 (:method Case3 :precondition (and (destination ?p - person ?c1 - city))
							:tasks ((debark ?p ?a ?c1) (transport-person ?p ?c)))
			 (:method Case4 :precondition (and (at ?p - person ?c1 - city) (at ?a - aircraft ?c2 - city) (different ?c1 ?c2))
							:tasks ((mover-avion ?a ?c2 ?c1) (transport-person ?p ?c)))
			 (:method Case5 :precondition (and (in ?p - person ?a - aircraft) (at ?a - aircraft ?c1 - city) (destination ?p - person ?c2 - city) (different ?c1 ?c2))
							:tasks ((mover-avion ?a ?c1 ?c2) (transport-person ?p ?c2)))
			 (:method Case6	:precondition (and (in ?p - person ?a1 - aircraft) (different ?a1 - aircraft ?a2 - aircraft))
							:tasks ((debark ?p ?a1 ?c1) (transport-person ?p ?c)))
)

(:task mover-avion
 :parameters (?a - aircraft ?c1 - city ?c2 -city)
			 (:method fuel-suficiente-fast :precondition (and (hay-fuel-fast ?a ?c1 ?c2) (different ?c1 ?c2) (available-duration-fast ?a ?c1 ?c2) (available-fuel-fast ?a ?c1 ?c2)) 
										   :tasks((zoom ?a ?c1 ?c2)))
			 (:method fuel-suficiente-slow :precondition (and (hay-fuel-slow ?a ?c1 ?c2) (different ?c1 ?c2) (available-duration-slow ?a ?c1 ?c2) (available-fuel-slow ?a ?c1 ?c2))
										   :tasks((fly ?a ?c1 ?c2)))
			 (:method no-fuel-suficiente-fast :precondition (and (not (hay-fuel-fast ?a ?c1 ?c2)) (different ?c1 ?c2)) 
										      :tasks((refuel ?a ?c1) (mover-avion ?a ?c1 ?c2)))
			 (:method no-fuel-suficiente-slow :precondition (and (not (hay-fuel-slow ?a ?c1 ?c2)) (different ?c1 ?c2))
										      :tasks((refuel ?a ?c1) (mover-avion ?a ?c1 ?c2)))
)

(:task board-all
 :parameters (?c - city)
			 (:method Case1 :precondition (and (at ?p - person ?c1 - city) (at ?a - aircraft ?c1 - city) (not (destination ?p - person ?c1 - city)))
							:tasks ((board ?p ?a ?c1) (board-all ?c)))
			 (:method Case2 :precondition ()
						    :tasks ())
)
 
(:import "Primitivas-Zenotravel.pddl"))