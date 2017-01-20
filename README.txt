-------------------------------------------README-------------------------------------------

This program solves the puzzle Rush Hour using graph theory and more specificly dijkstra


Libraries neeeded :

No special libraries are needed, but make sure you have numpy, matplotlib.

Execution :

python3 dijkstrahour.py -i filepath -r RHC/RHM -a All/Some -p Yes/No
Exemple : python3 dijkstrahour.py -i "./puzzles/Expert/jam40.txt" -a Some  -r RHC -p Yes

There is two modes to solve the puzzle with : RHC vs RHM

+  RHC finds out the solution of the puzzle while optimizing the numbers of cases covered when we move

+  RHM finds out the solution of the puzzle while optimizing the number of movements ( doens't care if we moved the car by one case or two cases or more)

Licence :

Read the LICENCE file.
