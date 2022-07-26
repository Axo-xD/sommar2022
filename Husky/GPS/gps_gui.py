from tkinter import *
from tkinter.ttk import Labelframe
import tkintermapview as tkmap
import customtkinter as ctk
# https://github.com/TomSchimansky/TkinterMapView

def default_view():
    print("default_view()")
    map_wig.set_position(61.4587735, 5.8875730) # Set the center of the map. (latitude, longitude)
    map_wig.set_zoom(18)
def goal_point(coordinates):
    print("goal_point()")
    print("Add goal point: ", coordinates)
    if len(goal_point_list) > 0:
        clear_markers()
    goal_point_list.append(map_wig.set_marker(coordinates[0],coordinates[1], "Goal"))
    boxlat.delete(0, END)
    boxlon.delete(0, END)
    boxlat.insert(0,coordinates[0])
    boxlon.insert(0,coordinates[1])
def set_goal_point():
    print("set_goal_point()")
    print(boxlat.get())
    try:
        lat = float(boxlat.get())
    except:
        boxlat.delete(0, END)
        boxlat.insert(0, "Invalid input")
        return
    try:
        lon = float(boxlon.get())
    except:
        boxlon.delete(0, END)
        boxlon.insert(0, "Invalid input")
        return
    coord = (float(boxlat.get()), float(boxlon.get()))
    goal_point(coord)
def clear_markers():
    print("clear_markers()")
    if len(goal_point_list) > 0:
        print("List was not empty")
        for i in goal_point_list:
            i.delete()
        goal_point_list.clear()
        boxlat.delete(0, END)
        boxlon.delete(0, END)
    else:
        print("List was empty")
def toggle_map_sat():
    print("toggle_map_set()")
    print(map_wig.tile_server)
    if map_wig.tile_server == "https://a.tile.openstreetmap.org/{z}/{x}/{y}.png":
        print("Map set to satellite")
        map_wig.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)
    else:
        print("Map set to map")
        map_wig.set_tile_server("https://a.tile.openstreetmap.org/{z}/{x}/{y}.png")

root = Tk()
root.title("Husky GPS")
root.geometry("1200x900")

bot_longitude = 50.00001
bot_latitude = 6.00001

# Map
mapframe = Labelframe(root)
mapframe.pack(pady=10, side=TOP)
goal_point_list = []
marker_list = []
map_wig = tkmap.TkinterMapView(mapframe, width=1150, height=600) # Create a map view object.
default_view()
map_wig.pack()
map_wig.add_right_click_menu_command("Set goal point", goal_point, pass_coords=True)

inputframe = Labelframe(root)
inputframe.pack(pady=10)

but1 = Button(inputframe, text="Default view", command=default_view)
but1.grid(row=1, column=0, sticky="ew")
but3 = Button(inputframe, text="Clear markers", command=clear_markers)
but3.grid(row=2, column=0, sticky="ew")
but4 = Button(inputframe, text="Toggle map/satellite", command=toggle_map_sat)
but4.grid(row=3, column=0, sticky="ew")

labelgoal = Label(inputframe, text="Goal data:")
labelgoal.grid(row=0, column=1,padx=100)
but4 = Button(inputframe, text="Set goal point", command=set_goal_point)
but4.grid(row=1, column=1, sticky="ew")
boxlat = Entry(inputframe)
boxlat.grid(row=2, column=1, sticky="ew")
boxlon = Entry(inputframe)
boxlon.grid(row=3, column=1, sticky="ew")

lablebotframe = Label(inputframe, text="GPS bot")
lablebotframe.grid(row=0, column=2, padx=100)
labellat = Label(inputframe, text=("Latitude: {}").format(bot_latitude))
labellat.grid(row=1, column=2)
labellon = Label(inputframe, text=("Longitude: {}").format(bot_longitude))
labellon.grid(row=2, column=2)



## Buttons
#buttonframe = Labelframe(root)
#buttonframe.pack(pady=10, side=LEFT)
## Reset view
#button1 = Button(buttonframe, text="Reset view", command = default_view)
#button1.pack()
## Clear markers
#button2 = Button(buttonframe, text="Clear markers", command=clear_markers)
#button2.pack()
## Button 3
#button3 = Button(buttonframe, text="Toggle map/sat", command=toggle_map_sat)
#button3.pack()
#
## Text
#textframe = Labelframe(root)
#textframe.pack(pady=10)
#
## Data frame
#dataframe = Labelframe(root)
#dataframe.pack(pady=10)
## Data frame name
#dataframe_name = Label(dataframe, text="Goal position")
#dataframe_name.pack(side=TOP)
## Position data for goal point
##Longitude
#dataframe_longitude = Labelframe(dataframe)
#dataframe_longitude.pack(side=TOP)
#dataframe_longitude_value = Label(dataframe_longitude, text="Longitude:")
#dataframe_longitude.pack(side=LEFT)
#dataframe_longitude_value = Label(dataframe, text="NA")
#dataframe_longitude_value.pack(side=RIGHT)
##Latitude
#dataframe_latitude = Label(dataframe, text="Latitude:")
#dataframe_latitude.pack(side=LEFT)
#dataframe_latitude_value = Label(dataframe, text="NA")
#dataframe_latitude_value.pack(side=RIGHT)

root.mainloop()
