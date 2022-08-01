from tkinter import *
import tkintermapview as tkmap
import customtkinter as ctk

'''
https://github.com/TomSchimansky/TkinterMapView
https://github.com/TomSchimansky/CustomTkinter

Tkinter Map View used to display the map. CustomTkinter used to create the GUI elements and improve the look from Tkinter.

'''

ctk.set_appearance_mode("Dark")  # Modes: "System" (standard), "Dark", "Light"
ctk.set_default_color_theme("blue")  # Themes: "blue" (standard), "green", "dark-blue"

class GPS_husky(ctk.CTk):
    def __init__(self,
                title="Husky GPS", 
                geometry="1200x900", # Window "{With}x{Hight}"
                map_width=1150, 
                map_height=600, 
                default_pos=(61.4587735, 5.8875730), 
                default_zoom=18, 
                husky_pos=(61.4587532, 5.8876001),
                number_of_markers = 4): # Maximum number of goal points
        super().__init__() # Call the parent class constructor.

        self.title(title)
        self.geometry(geometry)

        self.default_pos = default_pos
        self.default_zoom = default_zoom
        self.husky_pos = husky_pos

        self.marker_list = []

        self.marker_path = None
        self.number_of_markers = number_of_markers

        # ======= Frames ======= #
        # Frame for the map
        self.mapframe = ctk.CTkFrame(master=self)
        self.mapframe.pack(pady=10, side=TOP)
        # Frame for the buttons
        self.dataframe = ctk.CTkFrame(master=self)
        self.dataframe.pack(pady=10, side=TOP)

        # ===== Map Wiget ===== #
        self.map_wig = tkmap.TkinterMapView(self.mapframe, width=map_width, height=map_height, corner_radius=10)
        self.map_wig.pack()
        self.__default_view()
        # Right click options #
        self.map_wig.add_right_click_menu_command("Set goal point", self.set_goal, pass_coords=True)

        # ===== Data Wigets ===== #
        # Buttons #
        self.butDef = ctk.CTkButton(self.dataframe, text="Default view", command=self.__default_view)
        self.butDef.grid(row=1,column=0,pady=5,padx=5)
        self.butClear = ctk.CTkButton(self.dataframe, text="Clear markers", command=self.__clear_markers)
        self.butClear.grid(row=2,column=0,pady=5,padx=5)
        self.butMaps = ctk.CTkButton(self.dataframe, text="Satellite", command=self.__toggle_maps)
        self.butMaps.grid(row=3,column=0,pady=5,padx=5)
        self.butDraw = ctk.CTkButton(self.dataframe, text="Draw path", command=self.__draw_path)
        self.butDraw.grid(row=4,column=0,pady=5,padx=5)
        # Goal #
        self.goalText = ctk.CTkLabel(self.dataframe, text="Goal").grid(row=0,column=1,pady=5,padx=5)
        self.goallat = ctk.CTkEntry(self.dataframe)
        self.goallat.grid(row=1,column=1,pady=5,padx=5)
        self.goallon = ctk.CTkEntry(self.dataframe)
        self.goallon.grid(row=2,column=1,pady=5,padx=5)
        self.butGoal = ctk.CTkButton(self.dataframe, text="Set new goal", command=self.__input_goal)
        self.butGoal.grid(row=3,column=1,pady=5,padx=5)
        # Husky position #
        self.huskyText = ctk.CTkLabel(self.dataframe, text="Husky location").grid(row=0,column=2,pady=5,padx=5)
        self.botlattext = ctk.CTkLabel(self.dataframe, text=("Latitude : {:.7f}").format(self.husky_pos[0]))
        self.botlattext.grid(row=1,column=2,pady=5,padx=5)
        self.botlontext = ctk.CTkLabel(self.dataframe, text=("Longitude : {:.7f}").format(self.husky_pos[1]))
        self.botlontext.grid(row=2,column=2,pady=5,padx=5)
        # Position marker #
        self.botmarker = self.map_wig.set_marker(self.husky_pos[0],self.husky_pos[1], marker_color_circle="black", marker_color_outside="gray40")
        # Bot view #
        self.botview = ctk.CTkCheckBox(self.dataframe, text="Bot view")
        self.botview.grid(row=3,column=2,pady=5,padx=5)

    # ======= Public Functions ======= #
    def set_bot_pos(self,cords): # Update the bot position. Also mapview and path if active
        # Update the bot position
        self.botlattext.configure(text=("Latitude : {:.7f}").format(cords[0]))
        self.botlontext.configure(text=("Longitude : {:.7f}").format(cords[1]))
        self.botmarker.delete()
        self.botmarker = self.map_wig.set_marker(cords[0],cords[1], marker_color_circle="black", marker_color_outside="gray40")
        # Follow the bot
        if self.botview.get():
            self.map_wig.set_position(cords[0],cords[1])
        # Update the path
        if self.marker_path is not None:
            self.marker_path.delete()
            self.marker_path = None
            self.__draw_path()
    def set_goal(self,cords): # Set goal point up to a limit. The limit is set in the constructor.
        self.marker_list.append(self.map_wig.set_marker(cords[0],cords[1]))
        while len(self.marker_list) > self.number_of_markers: # Remove the oldest marker if the limit is reached
            self.marker_list[0].delete()
            self.marker_list.pop(0)
    def remove_goal(self): # Removes the first goal point in the list
        self.marker_list[0].delete() # Delete the marker from the map
        self.marker_list.pop(0) # Remove the marker from the list
    def get_current_goal(self): # Returns the coordinates next goal point
        if len(self.marker_list)>0:
            return self.marker_list[0].position

    # ======= Private Functions ======= #
    def __default_view(self):
        self.map_wig.set_position(self.default_pos[0],self.default_pos[1])
        self.map_wig.set_zoom(self.default_zoom)
    def __clear_markers(self):
        if len(self.marker_list)>0:
            for i in self.marker_list:
                i.delete()
            self.marker_list.clear()
    def __toggle_maps(self):
        if self.map_wig.tile_server == "https://a.tile.openstreetmap.org/{z}/{x}/{y}.png": # This links to the default map from osm
            self.map_wig.set_tile_server("https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)
            self.butMaps.configure(text="Map") # Change button text
        else:
            self.map_wig.set_tile_server("https://a.tile.openstreetmap.org/{z}/{x}/{y}.png")
            self.butMaps.configure(text="Satellite") # Change button text
    def __input_goal(self):
        try:
            lat = float(self.goallat.get())
        except:
            self.goallat.delete(0, END)
            self.goallat.insert(0, "Invalid input")
            return
        try:
            lon = float(self.goallon.get())
        except:
            self.goallon.delete(0, END)
            self.goallon.insert(0, "Invalid input: ")
            return
        coord = (float(lat), float(lon))
        self.set_goal(coord)
    def __draw_path(self):
        pos_list = []
        pos_list.append(self.botmarker.position)
        for i in self.marker_list:
            pos_list.append(i.position)
        if self.marker_path is not None:
            self.marker_path.delete()
            self.marker_path = None
            return
        if len(pos_list)>1:
            self.marker_path = self.map_wig.set_path(pos_list, color="red")
        

if __name__ == "__main__":
    gps_husky=GPS_husky()
    gps_husky.mainloop()