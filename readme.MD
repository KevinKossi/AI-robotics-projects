# AIndjes famillie 

## Beschrijving oorspronkelijk project
- De mamaEend (voorste bot ) navigeert in een al gedefinieerde map, terwijl het de kortste route neemt.
- de babyEendjes volgen de mamaEend naar de bestemming
  
- voor de navigatie wordt er gebruik gemaakt van Rviz. We implementeren het Dijkstra algorithm om de kortste route te volgen. 
- de baby eendjes volgen de mama eend op basis van haar coordinaten die zij continue publisht.
- elke baby eend zit in een rij en ze mogen mekaar niet voorbij steken 
  
## Beschrijving gerealiseerd project
- mama eend kan zich navigeren in een omgeving door middel van het dijkstra algorithm. de kortste route wordt altijd op voorhand berekend.
- mamaeend kan obstakels ontwijken. ze gaat links of recht afwijken 


## Verklaringen indien er werd afgeweken van het oorspronkelijke project
RVIZ, het navigation systeem gaf zeer veel complicaties met haar mee. Een daarvan zijnde dat ik geen navigation kon opstarten als er meer dan 1 bot wordt verbonden met een rosserver. dit zorgt ervoor dat ik geen babyEendjes kan implementeren waardoor ik mijn project nog enkel kon verder ontiwkkelen met de mama-eend.



## Tijdslijn van realisatie versus oorspronkelijke planning zodat kan nagegaan worden hoe uw tijdsplanning werd gerespecteerd.
- eerste maand van de tweede semsester
  
  ik was bezig met het deftig implementeren van het navigatie systeem voor de mamaeend 

- Paasvakantie
begonnen met het schrijven van scripts voor de baby eendjes. door middel van CV2 Package implementter ik camera detectie voor de bots.
Eveneens experimenteer ik ook met de namespaces

- na de paasvakantie
  Na overleg met de leerkracht, is een een beslissing gekomen om het project aan te passen. de focus ligt nu bij de mamaeend.
  Er wordt noch namespace gebruikt, noch script voor de baby eendjes.
  De coordinaten worden aangepast op een manier waarbij het navigatiesysteem werkt voor de schoolmap. Het dijkstra algortime creert een grafiek van nodes om een visueel beeld te verkrijgen van de map voor de kortste route 


## how to use the package ? 

1. zorg dat de turtlebot verbonden is met de rosserver
2. start RVIZ op met de schoolmap
3. typ commando " python3 graph.py" 

mkdir -p scripts
mv your_repository/your_python_script.py scripts
chmod +x scripts/your_python_script.py
