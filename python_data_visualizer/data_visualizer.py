from tkinter import *
from tkinter import ttk
import re
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)

root = Tk()
root.title("IMU Data Visualizer")
root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)

StrPitch    = StringVar()
StrRoll     = StringVar()
StrYaw      = StringVar()
StrTime     = StringVar()
StrNumber   = StringVar()

NumbList    = list()
PitchList   = list()
RollList    = list()
YawList     = list()


def read_callback():
    Header          = textentry.get("1.0", "1.end")
    Data            = textentry.get("2.0", "end")
    
    HeaderNumbers   = re.findall(r'-?\d+',Header)
    StrPitch.set(HeaderNumbers[0])
    StrRoll.set(HeaderNumbers[1])
    StrYaw.set(HeaderNumbers[2])
    StrTime.set(HeaderNumbers[3])
    StrNumber.set(HeaderNumbers[4])
    
    Data = Data.replace("<\\n>", "")
    splitData = Data.splitlines()
    splitData = list(filter(None, splitData))
    
    NumbList.clear
    PitchList.clear
    RollList.clear
    YawList.clear
    for point in splitData:
        pointData = re.findall(r'-?\d+', point)
        NumbList.append(int(pointData[0]))
        PitchList.append(int(pointData[1])*180/32768)
        RollList.append(int(pointData[2])*180/32768)
        YawList.append(int(pointData[3])*180/32768)
    

def visualize_callback():
    TimeList = [int(StrTime.get())* i for i in NumbList]
    
    pitchplot.cla()
    rollplot.cla()
    yawplot.cla()
    
    pitchplot.plot(TimeList,PitchList, "b")   
    pitchplot.step([TimeList[0], TimeList[-1]],[0,int(StrPitch.get())], "r")
    rollplot.plot(TimeList,RollList, "b")   
    rollplot.step([TimeList[0], TimeList[-1]],[0,int(StrRoll.get())], "r")
    yawplot.plot(TimeList,YawList, "b")   
    yawplot.step([TimeList[0], TimeList[-1]],[0,int(StrYaw.get())], "r")
    
    
    canvas.draw()
    
    
    






mainframe = ttk.Frame(root, padding = (3, 3, 12, 12))
mainframe.grid(column = 0, row  = 0, sticky = "NWSE")
mainframe.rowconfigure(1, weight=1)
mainframe.columnconfigure(0, weight=1)
mainframe.columnconfigure(1, weight=1)


controlframe = ttk.Frame(mainframe)
controlframe.grid(column = 0, row = 0, columnspan =2, sticky = (N, W, S, E))

textframe = ttk.Frame(mainframe)
textframe.grid(column = 0, row = 1, sticky = (N, W, S, E), padx =10, pady =10)
textframe.rowconfigure(0, weight=1)
textframe.columnconfigure(0, weight=1)


dataframe = ttk.Frame(mainframe)
dataframe.grid(column = 1, row = 1, sticky = (N, W, S, E), padx = 10, pady =10)
dataframe.rowconfigure(0, weight=1)
dataframe.columnconfigure(0, weight=1)

userinfoframe = ttk.Frame(mainframe)
userinfoframe.grid(column = 0, row = 2, columnspan = 2, sticky = (N, W, S, E))



#controlframe
readbutton = ttk.Button(controlframe, text = "Read", command = read_callback)
readbutton.grid(column = 0, row = 0)

pitchframe = ttk.Frame(controlframe)
pitchframe.grid(column = 1, row = 0, sticky = (W, E))

rollframe = ttk.Frame(controlframe)
rollframe.grid(column = 2, row = 0, sticky = (W, E))

yawframe = ttk.Frame(controlframe)
yawframe.grid(column = 3, row = 0, sticky = (W, E))

timeframe = ttk.Frame(controlframe)
timeframe.grid(column = 4, row = 0, sticky = (W, E))

numberframe = ttk.Frame(controlframe)
numberframe.grid(column = 5, row = 0, sticky = (W, E))

visualizebutton = ttk.Button(controlframe, text = "Visualize", command= visualize_callback)
visualizebutton.grid(column = 6, row = 0, sticky =(W, E))



pitchlabel = ttk.Label(pitchframe, text = "Pitch =")
pitchlabel.grid(column = 0, row = 0)

pitchentry = ttk.Entry(pitchframe, textvariable = StrPitch)
pitchentry.grid(column = 1, row = 0)



rolllabel = ttk.Label(rollframe, text = "Roll =")
rolllabel.grid(column = 0, row = 0)

rollentry = ttk.Entry(rollframe, textvariable = StrRoll)
rollentry.grid(column = 1, row = 0)



yawlabel = ttk.Label(yawframe, text = "Yaw =")
yawlabel.grid(column = 0, row = 0)

yawentry = ttk.Entry(yawframe, textvariable = StrYaw)
yawentry.grid(column = 1, row = 0)



timelabel = ttk.Label(timeframe, text = "Time between Measurements (ms) =")
timelabel.grid(column = 0, row = 0)

timeentry = ttk.Entry(timeframe, textvariable = StrTime)
timeentry.grid(column = 1, row = 0)


numberlabel = ttk.Label(numberframe, text = "Number of Measurements =")
numberlabel.grid(column = 0, row = 0)

numberentry = ttk.Entry(numberframe, textvariable = StrNumber)
numberentry.grid(column = 1, row = 0)





#textframe
textentry = Text(textframe)
textentry.grid(column = 0, row = 0, sticky = (N, S , W, E))






#dataframe
figure = Figure(dpi = 100)

pitchplot = figure.add_subplot(3,1,1)
pitchplot.set_title(label = "Pitch")

pitchplot.set_ylabel("Angle (°)")

rollplot = figure.add_subplot(3,1,2)
rollplot.set_title("Roll")

rollplot.set_ylabel("Angle (°)")

yawplot = figure.add_subplot(3,1,3)
yawplot.set_title("Yaw")
yawplot.set_xlabel("Time (ms)") 
yawplot.set_ylabel("Angle (°)")

canvas = FigureCanvasTkAgg(figure, master = dataframe)
canvas.get_tk_widget().pack(fill = "both", expand = True)

toolbar = NavigationToolbar2Tk(canvas,dataframe)
toolbar.pack()

toolbar.update()
canvas.draw()




root.mainloop()