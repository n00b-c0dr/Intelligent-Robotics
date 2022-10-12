## Sumit 
## 19317


###  Q1

Total Time Required to Cover a Cell(T) = Total Time to Moving Forward+Total Time of Turning

T = Total Distace Travelled Forward/Forward speed + Number of Turns*Time Taken on each Turn

Total Distace Travelled Forward/Forward speed + Number of Turns*(pi/(2*rotation speed)

Rotation speed = 4.856 rad/s

Forward speed = -0.128 m/s
​


T_cell0 = 389.25s

T_cell1 = 3.28s

T_cell2 = 88.02s

T_cell3 = 1.72s

T_cell4 = 53.90s

T_cell5 = 61.70s

T_cell6 = 80.09s

T_cell7 = 62.97

T  = 740.96


### Q2

Old Start: C0 -> C1 -> C2 -> C3 -> C4 -> C3 -> C2 -> C5 -> C6 -> C7 

New Start: C5 -> C6 -> C7 -> C0 -> C1 -> C2 -> C3 -> C4

T' <= T


### Q5 

I tried implementing a simple algorithm for persuader evader game where the we define a few camera positions as such the robot can cover scanning the whole map if it goes to all these positions but this did not work because i could not code the conditions for when the persuader sees the evader. This is probabily not an optimal algorithm but it does promise complete in a long time.  
If we use 2 persuader instead of 1, we can send both of them to different areas so as to cover as much area as possible at the same time. So, this will result in faster capturing of the evader.


### Q6

We can make 2 persuader robots with different algorithms to get different paths for both of them. To make sure their paths do not cross eath othe, we can have a base persuader which plans its path regardless of how the other persuader is moving and we can code the other persuader's path planning such that it will always try to avoid the base persuader.

