


SUBSCRIBE: /pose that will have pose of current location of Cabot from localizer node geometry_msgs/Pose2D

PUBLISH: /leftval - float32 left motor velocity
	/rightval - float32 right motor velocity


-User has to manually publish current position to /pose topic 

-When localization is integrated, current location will be continually updated by subsribing to the node 

-Grabs waypoints from the .yaml file in the config directory

-MIGHT NEED TO CHANGE MATH EQUATIONS AFTER TESTING (map velocities differently)

### Running Generate Angles tool
There is a file named generateAngles.py in the /config folder. This tool takes in a file of x and y pairs and outputs a .YAML file containing original x, y values as well as the angle between the vectors (theta).

1st argument: name of file that contains coordinate pairs separated by a newline. 
2nd argument: name of the file to write to. This will be a .YAML file (DO NOT INCLUDE ".YAML" IN FILE NAME).
3rd argument (optional): "d" outputs theta in degrees. By default, program outputs radians.
```
python generateAngles.py <coordinates_filename> <output_filename> <d>
```

