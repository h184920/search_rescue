Her kan vi holde styr på hva som er igjen av biter. også for å kommunisere slik at vi vet hvem som gjør hva videre.
Her kan alle ligge til og fjerne oppgaver fortkommende: 


Igjen å gjøre:

    For koding:

    - Endre navn på ting

    - Superkontroller: Koble opp superkontroller logikk for styring av robottene. #Freddy og #jonas

        ##-1. subscriptions til tb_i/aruco_marker

        ##-2. opprette klient til poeng-server
  
        ##-3. publishers til tb_i/filtered_detection 

    - Koordinator: implementere 'koordinator.py' # elias og jonas holder på 

        -1. subscriptions til tb_i/filtered_detection (ArucoMarker message type)

        -2. punkt-generatorer fra OccupancyGrid <-- '/map'

        -3. action-klienter til Bug2 

            |-> Om stor flamme detektert, send 2 robotter til posisjonen den er på (får aruco   markeren sin posisjon og id ID fra topic)

    - Bug2: Ligge til Bug2 algoritme #Elias holder på
        
        - Koble til action client til super #Elias holder på

    For Rapporten: 

    - #Freddy holder på
    
    -    
    - #Brage + mathias experiments. Ligger beskrivelse av hva som bør være med i felles dokumentet.  
    
    - 

    - 


    Utenom:

        - Lage readme fil #Elias jobber med

        - lage launch fil for alle kjørende ros noder vi trenger, #elias kan fint fikse om ingen andre vil gjøre dette:) 

        - fjerne ubrukt kode/klasser/funksjoner

        - har vi mye tid så er det garrantert super behov for opptimalisering/gjøre robot mer smooth i bevegelsene. 