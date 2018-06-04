(define (problem zeno-0)

(:domain zeno-travel)

(:customization
	(= :time-format "%d/%m/%Y %H:%M:%S")
	(= :time-horizon-relative 2500)
	(= :time-start "05/06/2007 08:00:00")
	(= :time-unit :hours)
)

(:objects 
    p1 p2 p3 p4 p5 p6 p7 p8 p9 p10 p11 p12 p13 p14 p15 p16 p17 p18 p19 p20 - person
    Almeria Barcelona Bilbao Cadiz Cordoba Gibraltar Granada Huelva Jaen Madrid Malaga Sevilla - city
	a1 a2 - aircraft
)

(:init
	(at p1 Granada)
	(at p2 Granada)
	(at p3 Granada)
	(at p4 Granada)
	(at p5 Granada)
	(at p6 Granada)
	(at p7 Granada)
	(at p8 Granada)
	(at p9 Granada)
	(at p10 Granada)
	(at p11 Granada)
	(at p12 Granada)
	(at p13 Granada)
	(at p14 Granada)
	(at p15 Granada)
	(at a1 Cordoba)
	(at a2 Jaen)
	(= (distance Almeria Barcelona) 809)
	(= (distance Almeria Bilbao) 958)
	(= (distance Almeria Cadiz) 463)
	(= (distance Almeria Cordoba) 316)
	(= (distance Almeria Gibraltar) 339)
	(= (distance Almeria Granada) 162)
	(= (distance Almeria Huelva) 505)
	(= (distance Almeria Jaen) 220)
	(= (distance Almeria Madrid) 547)
	(= (distance Almeria Malaga) 207)
	(= (distance Almeria Sevilla) 410)
	(= (distance Barcelona Almeria) 809)
	(= (distance Barcelona Bilbao) 620)
	(= (distance Barcelona Cadiz) 1284)
	(= (distance Barcelona Cordoba) 908)
	(= (distance Barcelona Gibraltar) 1124)
	(= (distance Barcelona Granada) 868)
	(= (distance Barcelona Huelva) 1140)
	(= (distance Barcelona Jaen) 804)
	(= (distance Barcelona Madrid) 621)
	(= (distance Barcelona Malaga) 997)
	(= (distance Barcelona Sevilla) 1046)
	(= (distance Bilbao Almeria) 958)
	(= (distance Bilbao Barcelona) 620)
	(= (distance Bilbao Cadiz) 1058)
	(= (distance Bilbao Cordoba) 796)
	(= (distance Bilbao Gibraltar) 1110)
	(= (distance Bilbao Granada) 829)
	(= (distance Bilbao Huelva) 939)
	(= (distance Bilbao Jaen) 730)
	(= (distance Bilbao Madrid) 395)
	(= (distance Bilbao Malaga) 939)
	(= (distance Bilbao Sevilla) 933)
	(= (distance Cadiz Almeria) 463)
	(= (distance Cadiz Barcelona) 1284)
	(= (distance Cadiz Bilbao) 1058)
	(= (distance Cadiz Cordoba) 261)
	(= (distance Cadiz Gibraltar) 124)
	(= (distance Cadiz Granada) 296)
	(= (distance Cadiz Huelva) 214)
	(= (distance Cadiz Jaen) 330)
	(= (distance Cadiz Madrid) 654)
	(= (distance Cadiz Malaga) 240)
	(= (distance Cadiz Sevilla) 126)
	(= (distance Cordoba Almeria) 316)
	(= (distance Cordoba Barcelona) 908)
	(= (distance Cordoba Bilbao) 765)
	(= (distance Cordoba Cadiz) 261)
	(= (distance Cordoba Gibraltar) 294)
	(= (distance Cordoba Granada) 160)
	(= (distance Cordoba Huelva) 241)
	(= (distance Cordoba Jaen) 108)
	(= (distance Cordoba Madrid) 396)
	(= (distance Cordoba Malaga) 165)
	(= (distance Cordoba Sevilla) 143)
	(= (distance Gibraltar Almeria) 339)
	(= (distance Gibraltar Barcelona) 1124)
	(= (distance Gibraltar Bilbao) 1110)
	(= (distance Gibraltar Cadiz) 124)
	(= (distance Gibraltar Cordoba) 294)
	(= (distance Gibraltar Granada) 255)
	(= (distance Gibraltar Huelva) 289)
	(= (distance Gibraltar Jaen) 335)
	(= (distance Gibraltar Madrid) 662)
	(= (distance Gibraltar Malaga) 134)
	(= (distance Gibraltar Sevilla) 201)
	(= (distance Granada Almeria) 162)
	(= (distance Granada Barcelona) 868)
	(= (distance Granada Bilbao) 829)
	(= (distance Granada Cadiz) 296)
	(= (distance Granada Cordoba) 160)
	(= (distance Granada Gibraltar) 255)
	(= (distance Granada Huelva) 346)
	(= (distance Granada Jaen) 93)
	(= (distance Granada Madrid) 421)
	(= (distance Granada Malaga) 125)
	(= (distance Granada Sevilla) 252)
	(= (distance Huelva Almeria) 505)
	(= (distance Huelva Barcelona) 1140)
	(= (distance Huelva Bilbao) 939)
	(= (distance Huelva Cadiz) 214)
	(= (distance Huelva Cordoba) 241)
	(= (distance Huelva Gibraltar) 289)
	(= (distance Huelva Granada) 346)
	(= (distance Huelva Jaen) 347)
	(= (distance Huelva Madrid) 591)
	(= (distance Huelva Malaga) 301)
	(= (distance Huelva Sevilla) 95)
	(= (distance Jaen Almeria) 220)
	(= (distance Jaen Barcelona) 804)
	(= (distance Jaen Bilbao) 730)
	(= (distance Jaen Cadiz) 330)
	(= (distance Jaen Cordoba) 108)
	(= (distance Jaen Gibraltar) 335)
	(= (distance Jaen Granada) 93)
	(= (distance Jaen Huelva) 347)
	(= (distance Jaen Madrid) 335)
	(= (distance Jaen Malaga) 203)
	(= (distance Jaen Sevilla) 246)
	(= (distance Madrid Almeria) 547)
	(= (distance Madrid Barcelona) 621)
	(= (distance Madrid Bilbao) 395)
	(= (distance Madrid Cadiz) 654)
	(= (distance Madrid Cordoba) 396)
	(= (distance Madrid Gibraltar) 662)
	(= (distance Madrid Granada) 421)
	(= (distance Madrid Huelva) 591)
	(= (distance Madrid Jaen) 335)
	(= (distance Madrid Malaga) 532)
	(= (distance Madrid Sevilla) 534)
	(= (distance Malaga Almeria) 207)
	(= (distance Malaga Barcelona) 997)
	(= (distance Malaga Bilbao) 939)
	(= (distance Malaga Cadiz) 240)
	(= (distance Malaga Cordoba) 165)
	(= (distance Malaga Gibraltar) 134)
	(= (distance Malaga Granada) 125)
	(= (distance Malaga Huelva) 301)
	(= (distance Malaga Jaen) 203)
	(= (distance Malaga Madrid) 532)
	(= (distance Malaga Sevilla) 209)
	(= (distance Sevilla Almeria) 410)
	(= (distance Sevilla Barcelona) 1046)
	(= (distance Sevilla Bilbao) 933)
	(= (distance Sevilla Cadiz) 126)
	(= (distance Sevilla Cordoba) 143)
	(= (distance Sevilla Gibraltar) 201)
	(= (distance Sevilla Granada) 252)
	(= (distance Sevilla Huelva) 95)
	(= (distance Sevilla Jaen) 246)
	(= (distance Sevilla Madrid) 534)
	(= (distance Sevilla Malaga) 209)
	(= (fuel a1) 1000)
	(= (slow-speed a1) 10)
	(= (fast-speed a1) 20)
	(= (slow-burn a1) 1)
	(= (fast-burn a1) 2)
	(= (total-fuel-used a1) 0)
	(= (fuel-limit a1) 1500)
	(= (capacity a1) 1000)
	(= (refuel-rate a1) 1)
	(= (fuel a2) 30000000)
	(= (slow-speed a2) 10)
	(= (fast-speed a2) 20)
	(= (slow-burn a2) 1)
	(= (fast-burn a2) 2)
	(= (total-fuel-used a2) 0)
	(= (fuel-limit a2) 15000000)
	(= (capacity a2) 30000000)
	(= (refuel-rate a2) 1)
	(= (boarding-time) 1)
	(= (debarking-time) 1)
	(= (num-passenger a1) 0)
	(= (capacity-passenger a1) 5)
	(= (duration a1) 0)
	(= (max-duration a1) 300000)
	(= (num-passenger a2) 0)
	(= (capacity-passenger a2) 10)
	(= (duration a2) 0)
	(= (max-duration a2) 3000000)
	(destination p1 Barcelona)
	(destination p2 Almeria)
	(destination p3 Cordoba)
	(destination p4 Jaen)
	(destination p5 Madrid)
	(destination p6 Cordoba)
	(destination p7 Cordoba)
	(destination p8 Bilbao)
	(destination p9 Cordoba)
	(destination p10 Cordoba)
	(destination p11 Gibraltar)
	(destination p12 Gibraltar)
	(destination p13 Cordoba)
	(destination p14 Malaga)
	(destination p15 Cordoba)
)

(:tasks-goal :tasks(
	(transport-person p1 Barcelona)
	(transport-person p2 Almeria)
	(transport-person p3 Cordoba)
	(transport-person p4 Jaen)
	(transport-person p5 Madrid)
	(transport-person p6 Cordoba)
	(transport-person p7 Cordoba)
	(transport-person p8 Bilbao)
	(transport-person p9 Cordoba)
	(transport-person p10 Cordoba)
	(transport-person p11 Gibraltar)
	(transport-person p12 Gibraltar)
	(transport-person p13 Cordoba)
	(transport-person p14 Malaga)
	(transport-person p15 Cordoba)
))

)