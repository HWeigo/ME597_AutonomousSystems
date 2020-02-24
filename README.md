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
List all running nodes .   
```
rosnode list
```
List node's detail.
```
rosnode info /some_topic
```

List all topics 
```
rostopic list
```
List all the nodes publish or subcribe to the topic.  
```
rostopic info /some_topic
```
Show the data being published on the topic .
```
rostopic echo /some_topic
```
e.g.  
```
rostopic echo /distance
```

```
 rostopic pub /some_topic msg/MessageType "data:value"
 ```
