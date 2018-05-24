(define (domain Belkan)
	(:requirements :strips :typing :fluents)
	(:types Player NPC Item Zone Orientation Type Bag)
	(:functions
        (Cost)
        (Distance ?mapA ?mapB - Zone)
		(Points)
        (AmountOfPoints ?object - Item ?mob - NPC)
        (PlayerSpeciality ?pj - Player)
    )
	(:predicates
		(PositionPlayer ?pj - Player ?map - Zone)
		(PositionItem ?object - Item ?map - Zone)
		(PositionNPC ?mob - NPC ?map - Zone)
		(OrientationPlayer ?pj - Player ?cardinal - Orientation)
		(Conection ?mapA - Zone ?mapB - Zone ?cardinal - Orientation)
		(Pick ?object - Item ?pj - Player)
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
        :precondition (and (not (PositionPlayer ?pj ?mapB)) (PositionPlayer ?pj ?mapA) (Conection ?mapA ?mapB ?cardinal) (OrientationPlayer ?pj ?cardinal) (ZoneType ?mapB ?floor) (or (Free ?floor) (and (or (Pick ?object ?pj) (Contains ?object)) (ItemNecessary ?object ?floor))))
        :effect (and (not (PositionPlayer ?pj ?mapA)) (PositionPlayer ?pj ?mapB) (increase (Cost) (Distance ?mapA ?mapB)))
    )
	(:action Coger
        :parameters (?pj - Player ?object - Item ?map - Zone)
        :precondition (and (PositionPlayer ?pj ?map) (PositionItem ?object ?map) (EmptyHand ?pj) (= (PlayerSpeciality ?pj) 1))
        :effect (and (not (EmptyHand ?pj)) (Pick ?object ?pj) (not (PositionItem ?object ?map)))
    )
	(:action Dejar
        :parameters (?pj - Player ?object - Item ?map - Zone)
        :precondition (and (PositionPlayer ?pj ?map) (Pick ?object ?pj))
        :effect (and (not (Pick ?object ?pj)) (PositionItem ?object ?map) (EmptyHand ?pj))
    )
	(:action EntregarNPC
        :parameters (?pj - Player ?object - Item ?map - Zone ?mob - NPC)
        :precondition (and (PositionPlayer ?pj ?map) (PositionNPC ?mob ?map) (Pick ?object ?pj) (not (Pick Zapatillas ?pj)) (not (Pick Bikini ?pj)) (= (PlayerSpeciality ?pj) 2))
        :effect (and (not (Pick ?object ?pj)) (Has ?object ?mob) (EmptyHand ?pj) (increase (Points) (AmountOfPoints ?object ?mob)))
    )
    (:action EntregarPJ
        :parameters (?pj1 - Player ?object - Item ?map - Zone ?pj2 - Player)
        :precondition (and (PositionPlayer ?pj1 ?map) (PositionPlayer ?pj2 ?map) (Pick ?object ?pj1) (= (PlayerSpeciality ?pj1) 1))
        :effect (and (not (Pick ?object ?pj1)) (Pick ?object ?pj2) (EmptyHand ?pj1) (not (EmptyHand ?pj2)))
    )
	(:action Guardar
        :parameters (?pj - Player ?object - Item)
        :precondition (and (Pick ?object ?pj) (EmptyBag ?pj))
        :effect (and (Contains ?object) (not (Pick ?object ?pj)) (not (EmptyBag ?pj)) (EmptyHand ?pj))
    )
    (:action Sacar
        :parameters (?pj - Player ?object - Item)
        :precondition (and (Contains ?object) (EmptyHand ?pj))
        :effect (and (Pick ?object ?pj) (not (Contains ?object)) (not (EmptyHand ?pj)) (EmptyBag ?pj))
    )
)