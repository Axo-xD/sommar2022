from gui_gps_husky import GPS_husky

cord = [61.4587532, 5.8876001]
flagtest = True

def cordset():
    global flagtest
    cord[0] = cord[0]+0.00001
    cord[1] = cord[1]+0.00001
    test.set_bot_pos(cord)
    print(test.get_current_goal())
    if (cord[0]>=(61.4587532 + 0.0002)) and flagtest:
        test.remove_goal()
        flagtest = False
        print("Goal removed")
    test.after(500, cordset)

test = GPS_husky()
test.set_goal((61.4587532 + 0.0002, 5.8876001 + 0.0002))
test.set_goal((61.4587532 + 0.002, 6.00000))
test.get_current_goal()
test.after(500, cordset)
test.mainloop()