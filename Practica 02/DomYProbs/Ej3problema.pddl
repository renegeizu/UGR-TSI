(define (problem Ejercicio3)
	(:domain Belkan)
	(:OBJECTS 
		Princesa Principe Bruja Profesor DiCaprio - NPC
		Oscar Manzana Rosa Algoritmo Oro Bikini Zapatillas - Item
		Jugador - Player
		Map1 Map2 Map3 Map4 Map5 Map6 Map7 Map8 Map9 Map10 Map11 Map12 Map13 Map14 Map15 Map16 Map17 Map18 Map19 Map20 Map21 Map22 Map23 Map24 Map25 - Zone
		Norte Sur Este Oeste - Orientation
		Bosque Agua Precipicio Arena Piedra - Type
		Mochila - Bag
	)
	(:INIT
		(Conection Map1 Map2 Este)
		(Conection Map2 Map1 Oeste)
		(Conection Map2 Map3 Este)
		(Conection Map3 Map2 Oeste)
		(Conection Map3 Map4 Este)
		(Conection Map4 Map3 Oeste)
		(Conection Map4 Map5 Este)
		(Conection Map5 Map4 Oeste)
		(Conection Map1 Map6 Sur)
		(Conection Map6 Map1 Norte)
		(Conection Map6 Map11 Sur)
		(Conection Map11 Map6 Norte)
		(Conection Map11 Map16 Sur)
		(Conection Map16 Map11 Norte)
		(Conection Map21 Map16 Sur)
		(Conection Map16 Map21 Norte)
		(Conection Map5 Map10 Sur)
		(Conection Map10 Map5 Norte)
		(Conection Map10 Map15 Sur)
		(Conection Map15 Map10 Norte)
		(Conection Map15 Map20 Sur)
		(Conection Map20 Map15 Norte)
		(Conection Map20 Map25 Sur)
		(Conection Map25 Map20 Norte)
		(Conection Map21 Map22 Este)
		(Conection Map22 Map21 Oeste)
		(Conection Map22 Map23 Este)
		(Conection Map23 Map22 Oeste)
		(Conection Map23 Map24 Este)
		(Conection Map24 Map23 Oeste)
		(Conection Map24 Map25 Este)
		(Conection Map25 Map24 Oeste)
		(Conection Map3 Map8 Sur)
		(Conection Map8 Map3 Norte)
		(Conection Map8 Map13 Sur)
		(Conection Map13 Map8 Norte)
		(Conection Map13 Map18 Sur)
		(Conection Map18 Map13 Norte)
		(Conection Map18 Map23 Sur)
		(Conection Map23 Map18 Norte)
		(Conection Map11 Map12 Este)
		(Conection Map12 Map11 Oeste)
		(Conection Map12 Map13 Este)
		(Conection Map13 Map12 Oeste)
		(Conection Map13 Map14 Este)
		(Conection Map14 Map13 Oeste)
		(Conection Map14 Map15 Este)
		(Conection Map15 Map14 Oeste)
		(Conection Map7 Map12 Sur)
		(Conection Map12 Map7 Norte)
		(Conection Map12 Map17 Sur)
		(Conection Map17 Map12 Norte)
		(Conection Map9 Map14 Sur)
		(Conection Map14 Map9 Norte)
		(Conection Map14 Map19 Sur)
		(Conection Map19 Map14 Norte)
		(= (Distance Map1 Map2) 5)
		(= (Distance Map2 Map1) 5)
		(= (Distance Map2 Map3) 8)
		(= (Distance Map3 Map2) 8)
		(= (Distance Map3 Map4) 12)
		(= (Distance Map4 Map3) 12)
		(= (Distance Map4 Map5) 5)
		(= (Distance Map5 Map4) 5)
		(= (Distance Map1 Map6) 8)
		(= (Distance Map6 Map1) 8)
		(= (Distance Map6 Map11) 12)
		(= (Distance Map11 Map6) 12)
		(= (Distance Map11 Map16) 5)
		(= (Distance Map16 Map11) 5)
		(= (Distance Map21 Map16) 8)
		(= (Distance Map16 Map21) 8)
		(= (Distance Map5 Map10) 12)
		(= (Distance Map10 Map5) 12)
		(= (Distance Map10 Map15) 5)
		(= (Distance Map15 Map10) 5)
		(= (Distance Map15 Map20) 8)
		(= (Distance Map20 Map15) 8)
		(= (Distance Map20 Map25) 12)
		(= (Distance Map25 Map20) 12)
		(= (Distance Map21 Map22) 5)
		(= (Distance Map22 Map21) 5)
		(= (Distance Map22 Map23) 8)
		(= (Distance Map23 Map22) 8)
		(= (Distance Map23 Map24) 12)
		(= (Distance Map24 Map23) 12)
		(= (Distance Map24 Map25) 5)
		(= (Distance Map25 Map24) 5)
		(= (Distance Map3 Map8) 8)
		(= (Distance Map8 Map3) 8)
		(= (Distance Map8 Map13) 12)
		(= (Distance Map13 Map8) 12)
		(= (Distance Map13 Map18) 5)
		(= (Distance Map18 Map13) 5)
		(= (Distance Map18 Map23) 8)
		(= (Distance Map23 Map18) 8)
		(= (Distance Map11 Map12) 12)
		(= (Distance Map12 Map11) 12)
		(= (Distance Map12 Map13) 5)
		(= (Distance Map13 Map12) 5)
		(= (Distance Map13 Map14) 8)
		(= (Distance Map14 Map13) 8)
		(= (Distance Map14 Map15) 12)
		(= (Distance Map15 Map14) 12)
		(= (Distance Map7 Map12) 5)
		(= (Distance Map12 Map7) 5)
		(= (Distance Map12 Map17) 8)
		(= (Distance Map17 Map12) 8)
		(= (Distance Map9 Map14) 12)
		(= (Distance Map14 Map9) 12)
		(= (Distance Map14 Map19) 5)
		(= (Distance Map19 Map14) 5)
		(ZoneType Map1 Arena)
		(ZoneType Map2 Precipicio)
		(ZoneType Map3 Arena)
		(ZoneType Map4 Agua)
		(ZoneType Map5 Arena)
		(ZoneType Map6 Bosque)
		(ZoneType Map7 Piedra)
		(ZoneType Map8 Arena)
		(ZoneType Map9 Piedra)
		(ZoneType Map10 Precipicio)
		(ZoneType Map11 Piedra)
		(ZoneType Map12 Piedra)
		(ZoneType Map13 Arena)
		(ZoneType Map14 Piedra)
		(ZoneType Map15 Piedra)
		(ZoneType Map16 Precipicio)
		(ZoneType Map17 Piedra)
		(ZoneType Map18 Arena)
		(ZoneType Map19 Piedra)
		(ZoneType Map10 Bosque)
		(ZoneType Map20 Arena)
		(ZoneType Map21 Arena)
		(ZoneType Map22 Agua)
		(ZoneType Map23 Arena)
		(ZoneType Map24 Precipicio)
		(ZoneType Map25 Arena)
		(PositionItem Oscar Map7)
		(PositionItem Manzana Map17)
		(PositionItem Rosa Map9)
		(PositionItem Algoritmo Map19)
		(PositionItem Oro Map11)
		(PositionItem Bikini Map3)
		(PositionItem Bikini Map23)
		(PositionItem Zapatillas Map12)
		(PositionItem Zapatillas Map14)
		(PositionNPC Princesa Map1)
		(PositionNPC Principe Map5)
		(PositionNPC Bruja Map15)
		(PositionNPC Profesor Map21)
		(PositionNPC DiCaprio Map25)
		(PositionPlayer Jugador Map13)
		(OrientationPlayer Norte)
		(Free Piedra)
		(Free Arena)
		(ItemNecessary Bikini Agua)
		(ItemNecessary Zapatillas Bosque)
		(= (Cost) 0)
		(EmptyHand)
		(EmptyBag)
	)
	(:goal 
		(and (Has Oscar DiCaprio) (Has Rosa Princesa) (Has Manzana Bruja) (Has Algoritmo Profesor))
	)
	(:metric minimize (Cost))
)
