Her kan vi holde styr på hva som er igjen av biter. også for å kommunisere slik at vi vet hvem som gjør hva videre.
Her kan alle ligge til og fjerne oppgaver fortkommende:)  


Igjen å gjøre:

    For koding:

    - Superkontroller: Koble opp superkontroller logikk for styring av robottene.

        -1. subscriptions til tb_i/aruco_marker

        -2. opprette klient til poeng-server
  
        -3. publishers til tb_i/filtered_detection 

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
    
    - 

    - 
