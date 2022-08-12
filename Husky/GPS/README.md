# GPS GUI for Husky

## Class description

### Dependencies:

1. Tkinter
2. https://github.com/TomSchimansky/TkinterMapView
3. https://github.com/TomSchimansky/CustomTkinter


### Constructor
The constructor sets these variables
* title
    `string`
    This sets the title of the window. 
*   geometry
    `string`
    Sets the width and height of the window
* map_width/map_hight
    `int`
    Sets the width and height of the map widget.
* default_pos
    `tuple`
    Sets the default center coordinates of the map.
* default_zoom
    `int`
    Sets the default zoom level.
* husky_pos
    `tuple`
    Set the starting position of the husky
* number_of_markers
    `int`
    Set the maximum number of goal points.

There are five public functions in this class.

### set_bot_pos()

Argument: A tuple or list containing coordinates to one point on the map (latitude, longitude).
Used to update the position of the husky on the map.

### set_goal()

Argument: A tuple or list containing coordinates to one point on the map (latitude, longitude).
Add the argument to the list of goals and creates a new marker.
Note: If the number of goals exceeds`number_of_makers`, the oldest goal will be deleted. This is set in the constructor. 

### remove_goal()

Used to remove the goal oldest goal point added to the list. 

### get_current_goal()

When called return a tuple with the coordinates to the oldest goal point in the list of goals. 
This is the next goal for the robot.

### get_goal_list()

Returns a list of all goal points.
## Example
`main.py` is an example of useing the GPS_husky class.
## Known bugs
At different map zoom levels, the map does not always center on the correct position.
-   Fixed by setting the zoom level to int in map_widget.py. Pull request sent to TkinterMapView.

The map stops updating when map fading is activated.
-   Fixed by disabling map fading in map_widget.py.
```
def mouse_click(self, event):
    self.fading_possible = False  # disable fading while mouse is down
```

