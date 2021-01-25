# warmup_project

## Square

### Problem

The square behaviour required the turtle to move in a "more or less" square path. The approach I took is to utilize a timer to move forward some distance and rotate 90 degrees, where it is repeated for a total of 4 times. 

To traverse linearly, I utilized a `while` loop that checked if the traversed distance is less than the specified `edge_len` distance. The traversed distance is calcualted from the specified `speed` and the amount of time turtlebot has moved so far in this edge.

To rotate 90 degrees, I specified an angular velocity of 30 degrees per second, and utilized a `while` loop that checked if the current angle is less than the target 90 degree angle. The current angle is calculated using a timer multiplied by the angular speed. 

In order to minimize drifting, I decided to utilize a very slow speed of 0.1

### Code Structure

There is a class `SquareNode` that is constructed with a given `node` name, `publisder` name, `speed` integer, `edge_len` integer. 

There are 2 methods in this class:

- `run` runs the Node's main activity loop, including traversing a linear distance and rotating for a total of 4 times 
- `rotate90` rotates the turtle by 90 degrees


### gif recording

![Gif of square driving](./gifs/square.gif)


## Wall Follower

### Problem

The Wall Follower behaviour required the turtle to search for walls and follow along them while traveling at a approximately fixed distance. 

The approach I took is to first divide the sensor's range into areas (Front, Front-Left, Front-Right), and take the minimum distance in that area. Using these points, I had conditional statements that instructed the turtle to search for walls, follow walls, and turn left when necessary. The turtle attempts to keep the wall on its right side. 


### Code Explanation

The code consist of 1 WallFollower class that subscribes to the /scan output and handles turtle movement as a callback to the subscriber. For more specifics please check out the code's comments. 

The main logic of the callback is as follows: 

If nothing is in range of Front, Front-Left, Front-Right, then the turtle moves forward with minimal angular velocity.

Else if the wall is only on the turtle's right side, move forward and turn right to correct itself if it strays too far from the wall. 

Else if the turtle is too close to the right side, or hits a corner, then turn left. 

### gif recording 

![Gif of wall following](./gifs/wall_follower.gif)


## Person Follower

### Problem

The Person Follower behaviour required the turtle to search for the closest object and follow them while maintaining a safe distance. 

The approach I took is to utilize the scanner's range data and look for the closest object. Once an object has been found, I compare the direction of the closest object to the front of the turtle, and assign an angular velocity to steer the front of the turtle torwards the closest object. 

### Code Exaplanation

The code consists of 1 PersonFollower class that subscribes to the /scan output and handles turtle movement as a callback to the subscriber. For more specifics please check out the code's comments.

The main logic of the callback is as follows:

I first try to find the direction of the closest object via its index in the `ranges` array. I then check if the turtle is close enough to the object. 

If the turtle is close enough, then it tries to orient itself to face the object.

Otherwise, the turtle orients itself and moves towards the closest object.

### gif recording 
![Gif of person following](./gifs/person_follower.gif)


## Challenges

One challenge i faced during this project is to deal with noise. Due to noise, I cannot expect the turtle to make specific turns and travel along a specified axis. The way I overcame this is by minimizing the turtle's velocity to avoid excessive noise when accelerating/decellerating, and by tuning parameters like `follow_distance`. 

## Future Work

I'd like to try and introduce PID control to my turtle's behaviour so that it can correct itself more efficiently and stay on a more accurate path. From then on I'd like to increase the velocity of my turtle so that it no longer crawls slowly from one wall to the next.

## Takeaways 

1. Programming a robot is a lot harder than I imagined. At some point everyone encounters 2D turtle programs consisting of commands like move forward and rotate which will execute perfectly according to the program. However, Robotics is about real world systems where noise is constantly a problem and the program needs to make up for it. 

2. Use an IDE. There are ROS packages available to autocomplete code, which helps a lot when first starting out to learn RosPy. It definitely makes navigating the library effortless. 
