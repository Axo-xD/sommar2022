from gui_gps_husky import GPS_husky

cord = [61.4587532, 5.8876001]
flagtest = True
goal_list = []

def cordset():
    global flagtest
    global goal_list
    cord[0] = cord[0]+0.00001
    cord[1] = cord[1]+0.00001
    test.set_bot_pos(cord) # Update the position of the bot
    if (cord[0]>=(61.4587532 + 0.0002)) and flagtest:
        test.remove_goal()
        flagtest = False
    goal_list = test.get_goal_list() # Get the list of goal points
    for i in range(len(goal_list)): # Print the list of goal points
        print(goal_list[i])
    test.after(200, cordset)

test = GPS_husky()  # Create a new instance of the class
test.set_goal((61.4587532 + 0.0002, 5.8876001 + 0.0002))
test.set_goal((61.4587532 + 0.0002, 5.8870001))
test.get_current_goal()
test.after(1000, cordset) # Start the interupt loop
test.mainloop() # Start the mainloop