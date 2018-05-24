(define (domain Belkan)
	(:requirements :strips :typing :fluents)
	(:types Player NPC Item Zone Orientation Type Bag)
	(:functions
        (Cost)
        (Distance ?mapA ?mapB - Zone)
		(Points)
        (AmountOfPoints ?object - Item ?mob - NPC)
        (ObtainedItems ?pj - Player)
    )
	(:predicates
		(PositionPlayer ?pj - Player ?map - Zone)
		(PositionItem ?object - Item ?map - Zone)
		(PositionNPC ?mob - NPC ?map - Zone)
		(OrientationPlayer ?pj - Player ?cardinal - Orientation)
		(Conection ?mapA - Zone ?mapB - Zone ?cardinal - Orientation)
		(Pick ?object - Item)
		(Has ?object - Item ?mob - NPC)
		(Contains ?object - Item)
		(ZoneType ?map - Zone ?floor - Type)
		(ItemNecessary ?object - Item ?floor - Type)
		(Free ?floor - Type)
		(EmptyHand ?pj - Player)
		(EmptyBag ?pj - Player)
	)
	(:action GirarIzquierda
        :parameters (?pj - Player ?cardinal - Orientation)
        :precondition (and (OrientationPlayer ?pj ?cardinal))
        :effect 
        (and
            (when (and (OrientationPlayer ?pj Norte))
                (and (OrientationPlayer ?pj Oeste) (not (OrientationPlayer ?pj Norte)))
            )
            (when (and (OrientationPlayer ?pj Oeste))
                (and (OrientationPlayer ?pj Sur) (not (OrientationPlayer ?pj Oeste)))
            )
            (when (and (OrientationPlayer ?pj Sur))
                (and (OrientationPlayer ?pj Este) (not (OrientationPlayer ?pj Sur)))
            )
            (when (and (OrientationPlayer ?pj Este))
                (and (OrientationPlayer ?pj Norte) (not (OrientationPlayer ?pj Este)))
            )
        )
    )
	(:action GirarDerecha
        :parameters (?pj - Player ?cardinal - Orientation)
        :precondition (and (OrientationPlayer ?pj ?cardinal))
        :effect 
        (and
            (when (and (OrientationPlayer ?pj Norte))
                (and (OrientationPlayer ?pj Este) (not (OrientationPlayer ?pj Norte)))
            )
            (when (and (OrientationPlayer ?pj Oeste))
                (and (OrientationPlayer ?pj Norte) (not (OrientationPlayer ?pj Oeste)))
            )
            (when (and (OrientationPlayer ?pj Sur))
                (and (OrientationPlayer ?pj Oeste) (not (OrientationPlayer ?pj Sur)))
            )
            (when (and (OrientationPlayer ?pj Este))
                (and (OrientationPlayer ?pj Sur) (not (OrientationPlayer ?pj Este)))
            )
        )
    )
	(:action Ir
        :parameters (?pj - Player ?mapA - Zone ?mapB - Zone ?cardinal - Orientation ?floor - Type ?object - Item)
        :precondition (and (not (PositionPlayer ?pj ?mapB)) (PositionPlayer ?pj ?mapA) (Conection ?mapA ?mapB ?cardinal) (OrientationPlayer ?pj ?cardinal) (ZoneType ?mapB ?floor) (or (Free ?floor) (and (or (Pick ?object) (Contains ?object)) (ItemNecessary ?object ?floor))))
        :effect (and (not (PositionPlayer ?pj ?mapA)) (PositionPlayer ?pj ?mapB) (increase (Cost) (Distance ?mapA ?mapB)))
    )
	(:action Coger
        :parameters (?pj - Player ?object - Item ?map - Zone)
        :precondition (and (PositionPlayer ?pj ?map) (PositionItem ?object ?map) (EmptyHand ?pj))
        :effect (and (not (EmptyHand ?pj)) (Pick ?object) (not (PositionItem ?object ?map)))
    )
	(:action Dejar
        :parameters (?pj - Player ?object - Item ?map - Zone)
        :precondition (and (PositionPlayer ?pj ?map) (Pick ?object))
        :effect (and (not (Pick ?object)) (PositionItem ?object ?map) (EmptyHand ?pj))
    )
	(:action Entregar
        :parameters (?pj - Player ?object - Item ?map - Zone ?mob - NPC)
        :precondition (and (PositionPlayer ?pj ?map) (PositionNPC ?mob ?map) (Pick ?object) (not (Pick Zapatillas)) (not (Pick Bikini)))
        :effect (and (not (Pick ?object)) (Has ?object ?mob) (EmptyHand ?pj) (increase (Points) (AmountOfPoints ?object ?mob)) (increase (ObtaniedItems ?pj) (AmountOfPoints ?object ?mob)))
    )
	(:action Guardar
        :parameters (?pj - Player ?object - Item)
        :precondition (and (Pick ?object) (EmptyBag ?pj))
        :effect (and (Contains ?object) (not (Pick ?object)) (not (EmptyBag ?pj)) (EmptyHand ?pj))
    )
    (:action Sacar
        :parameters (?pj - Player ?object - Item)
        :precondition (and (Contains ?object) (EmptyHand ?pj))
        :effect (and (Pick ?object) (not (Contains ?object)) (not (EmptyHand ?pj)) (EmptyBag ?pj))
    )
)