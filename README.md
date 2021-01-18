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
