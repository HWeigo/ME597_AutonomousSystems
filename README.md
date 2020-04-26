# ME597_AutonomousSystems
## Command Reference

Run the whole script.   
```
cd ./launch
roslaunch autonomy launcher.launch
```
Run the test script (not actuating the motors).   
```
cd ./launch
roslaunch autonomy tester.launch
```
List rostopic
```
rostopic list
```
Display ultrasonic sensor's feedback value (0-1).
```
rostopic echo /distance
```
