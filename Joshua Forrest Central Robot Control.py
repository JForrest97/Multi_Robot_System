import math
import threading
import os
from math import pi
#The following modules are part of the SciPy environment, scipy.org
import numpy as np
import matplotlib.animation as animation
import matplotlib.pyplot as plt
#Serial module from https://pythonhosted.org/pyserial/index.html ,BSD license, (C) 2001-2017 Chris Liechti <cliechti@gmx.net>
import serial

Robots = {} #Dictionary of all Robots
lines = [] #List of Robot scan lines to be plotted
ParentReadings = {} #Dictionary containing unique ParentReading Class instances spaced every 10mm, same as Navigation Nodes
PathfindingNodes = {} #Dictionary of Navigation Nodes
#Map Plot Setup
fig = plt.figure()
ax = fig.add_subplot(1,1,1)
ax.set_ylim([0,1000])
ax.set_xlim([0,1000])
plt.title('Scanner Map')
plt.xlabel('x')
plt.ylabel('y')
#Robot Position Markers
RobotPositions = ax.scatter([], [], s=10, zorder = 3, c = 'g',marker = 'o')
RobotContactAreas = ax.scatter([], [], s=6000, zorder = 3, edgecolor = 'g',marker = 'o', facecolors = 'none')
lines.append(RobotPositions)
lines.append(RobotContactAreas)
#Reading Plot Setup
ConfirmedRobotReadings = ax.scatter([], [], s=2, zorder = 2, c = 'b')
lines2 = lines
lines2.append(ConfirmedRobotReadings)
#Traversal Node plot setup, Change to False to Hide the map
ShowTraverseMap = True
UnavailableArea = ax.scatter([],[], s=2, zorder =1, c = 'r')
AvailableArea = ax.scatter([],[], s=2, zorder =1, c = 'y')
lines3 = lines2
lines3.append(UnavailableArea)
lines3.append(AvailableArea)
#Class for weighted readings
class ParentReading:
    
    def __init__(self, position):
        self.position = position
        self.children = []
        self.displayedposition = position #position shown on the graph
        self.aliasedposition = position
        self.active = False
        self.DeactivationZone = []
    
    def UpdatePosition(self):
        a2 = 0
        b2 = 0
        for x in self.children:
            [a,b] = x
            a2 += a
            b2 += b
        self.aliasedposition = [round(a2/len(self.children)),round(b2/len(self.children))]
        self.displayedposition = self.aliasedposition

    def ResetPosition(self):
        self.displayedposition = self.position

    def activate(self):
        self.active = True
        for x in self.DeactivationZone:
            x.deactivate()

    def FindDeactivationZone(self):
        for x in range(-50,50,10):
            for y in range(-50,50,10):
                if abs(x^2+y^2) <=100^2:
                    xpos = x + self.position[0]
                    ypos = y + self.position[1]
                    try:
                        self.DeactivationZone.append(PathfindingNodes[xpos,ypos])
                    except KeyError:
                        continue
#Create parent readings
print("Creating Parent Readings...")
for x in range(101):
    for y in range(101):
        ParentReadings[x*10,y*10] = ParentReading([x*10,y*10])
#Node Class for pathfinding. Both this class and the function for A* pathfinding are based off code made by Nicholas Swift. Original Unmodified code can be found here https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        #(f=g+h) used for algorithm
        self.g = 0
        self.h = 0
        self.f = 0
        self.neighbours = []
        self.available = True

        
    def findneighbours(self):
        for new_position in [(0, -10), (0, 10), (-10, 0), (10, 0), (-10, -10), (-10, 10), (10, -10), (10, 10)]: # Adjacent squares
            # Get node position
            try:
                new_node = PathfindingNodes[new_position[0]+self.position[0],new_position[1]+self.position[1]]
                pathweight = (new_position[0]**2+new_position[1]**2)**0.5
                self.neighbours.append([new_node, pathweight])
            except KeyError:
                continue

    def deactivate(self):
        self.available = False
        self.PathCheck()

    def PathCheck(self):
        #trigger on deactivate, checks current robot paths to ensure that the node isnt on the current path
        for x in Robots:
            if self.position in Robots[x].CommandList:
                Robots[x].ReplanPath()   
#Create Pathfinding Node Grid
print("Creating Pathfinding Nodes...")
for x in range(101):
    for y in range(101):
        PathfindingNodes[x*10,y*10] = Node(position = [x*10,y*10])
#Find Each Nodes Neighbours
print("Calculating Node Neighbours...")
for x in PathfindingNodes.values():
    x.findneighbours()
#Calculate Reading Deactivation zones
print("Calculating Reading Deactivation Zones...")
for x in ParentReadings.values():
    x.FindDeactivationZone()
#Not Needed?
#activatednodes = [x.position for x in PathfindingNodes.values() if x.available]
#anodes = np.c_[np.array(activatednodes)]
#AvailableArea.set_offsets(anodes)

#String for menu
MenuString =(f"""Select Command:
1. Setup New Robot
2. Command Robot
3. List Robots
4. Begin Mapping
5. Reconnect
6. Navigation
7. Refine Map
8. Exit

Enter Menu at any time to return to here. 
Beginning Mapping will exit this menu.
""")
#Initial function for animation
def init():
    return lines

#A* pathfinding algorithm    
def AStarPathfinding(Robot = None, Target = None):
    #Create Start and End nodes (robot current position and target position)
    startx = round(Robot.x0, -1)
    starty = round(Robot.y0, -1)
    start_node = PathfindingNodes[startx,starty]
    start_node.g = start_node.h = start_node.f = 0
    end_node = PathfindingNodes[Target[0],Target[1]]
    end_node.g = end_node.h = end_node.f = 0
    #Initialise Open and Closed List
    open_list = []
    closed_list = []
    #Add start node to open list
    open_list.append(start_node)
    nodessearched = 0
    laststart = start_node
    #Loop algorithm until Target node found
    while len(open_list) > 0:
        #Pick Node with lowest f value
        current_node = min(open_list, key = lambda x:x.f)
        current_index = open_list.index(current_node)


        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            print("Path found")
            return path[::-1] # Return reversed path

        # Analyse children
        for child, weight in current_node.neighbours:
            #Ignore if child isnt available
            if child.available is False:
                continue
            #Ignore if child has alrady been analysed
            if child in closed_list:
                continue

            # Create the f, g, and h values
            child.g = current_node.g + weight
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h
            
            if child.parent is None:
                child.parent = current_node

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)
        nodessearched = nodessearched+1
#Custom Built class for robots
class Robot:
    #Initial Settings
    def __init__(self, name, port=None, baudrate = 115200, timeout = 0.5):
        Robots[name] = self
        #Objects for serial communication
        self.channel = serial.Serial()
        self.channel.port = port
        self.channel.baudrate = baudrate
        self.channel.timeout = timeout
        self.name = name
        self.x0 = 0.0 #robot x pos
        self.y0 = 0.0 #robot y pos
        self.position = [self.x0,self.y0]
        self.R = None #range reading from sensor
        self.th = None#current servo angle
        self.Orientation = None # current orientation angle of robot
        self.accelx = None
        self.accely = None
        self.TOF = None
        self.line, = ax.plot([], [], 'b', lw = 0.5, zorder = 3) 
        lines.append(self.line)
        self.speed = 222
        self.angularspeed = 160
        self.Target = []
        self.CommandList = [] 
        self.Commands = []
        self.Replan = False
        try:
            self.channel.open()
            print(f"Connected to {name}")
            threading.Thread(target = self.parse, args = (), daemon = True).start()
            print(f"Parse Thread Begun for {name}")
            #threading.Thread(target = self.Comms, args = ()).start()  Thread is slowing down everything else considerably
            #print(f"Command Thread Begun for {name}")
        except:
            print(f"Failed to Connect to {name}")      
            
    #Connect to Robot at specified port
    def connect(self, port, baudrate, timeout=0.5):
        self.channel.port = port
        self.channel.baudrate = baudrate
        self.channel.timeout = timeout
        try:
            self.channel.open()
            print("Connected")
        except:

            print("Failed to Open")
    #Disconnect from port
    def disconnect(self):
        try:
            self.channel.close()
            print("Disconnected")
        except:
            print("Failed to Disconnect")
    #Parse incoming Data
    def parse(self):
        id = ""
        message = ""
        NothingRecieved = 0
        while self.channel.is_open:
            Rx = self.channel.readline()
            if Rx:
                Message = str(Rx.decode("ASCII"))
                NothingRecieved = 0
                try:
                    (id, message) = Message.split(';')
                    self.dataRouting(id, message)
                except ValueError:
                    print(f"""Message Parsing Error: {Message}""")
            else:
                if NothingRecieved == 0:
                    print("Nothing Recieved....")
                    NothingRecieved = 1
                Rx = self.channel.readline()       
    #Command
    def command(self, cmd):
       msg = '<' + cmd + '>'
       self.channel.write(bytes(msg, encoding = 'ASCII'))
    #Route incoming data
    def dataRouting(self, id, message):
        if id == "map":
                (self.R, self.x0, self.y0, self.th, self.Orientation) = [float(s) for s in message.split(',', maxsplit = 5)]
                th2 = math.radians(self.th-30) #Converts angle sent to radians for math functions
                O = math.radians(self.Orientation)
                x2 = ((self.x0+(30*math.cos(O)))+40*math.cos(O+th2)) # x Position of Sensor Servo (x+rcos(0)) where r is distance from the center of the robot to the servo's center of rotation
                y2 = ((self.y0+(30*math.sin(O)))+40*math.sin(O+th2)) # y Position of Sensor Servo
                rx = x2+(200*math.cos(O+th2)) # x read range (x+rcos(0)+Rcos(th+0)) where R is sensor read range and th is current servo angle from the center line
                ry = y2+(200*math.sin(O+th2)) # y read range
                self.line.set_data([x2, rx], [y2, ry]) #Updates Position of Scan Line
                if self.R < 200: #If R is less than 200 a reading has been made, R defaults to 255 when no reading
                    rangex = x2+((self.R)*math.cos(O+th2))
                    rangey = y2+((self.R)*math.sin(O+th2))#Scatter Plot of Readings
                    rangex2 = int(round(rangex, -1))
                    rangey2 = int(round(rangey, -1))
                    if 0 <= rangex2 <= 1000 and 0 <= rangey2 <= 1000:
                        Parent = ParentReadings[rangex2,rangey2]
                        Parent.children.append([rangex,rangey])
                        Parent.activate()
                else:
                    if self.Replan:
                        newpath = AStarPathfinding(Robot = self, Target = self.Target)        
                        self.CommandList = newpath
                        self.Commands = self.PathParse(newpath)
                        self.Replan = False
        elif id == "Status": #Status messages from the robot
            print(f"\n{self.name}: {message}")
        elif id == "Acc": #IMU Data
            (self.accelx, self.accely) = [float(s) for s in message.split(',', maxsplit = 2)]
        elif id == "CMD":
            if len(self.Commands) > 0:
                nextcommand = self.Commands.pop(0)
                print(nextcommand)
                self.command(nextcommand)
        elif id == "TOF":
            self.TOF = message

    def ReplanPath(self):
        self.CommandList = []
        self.Commands = []
        self.command('X')
        self.Replan = True

#Converts Path returned from A* algorithm to Instructions for Robot 
    def PathParse(self, path):
        change = np.array([[0,0]])
        preve = 0
        prevr = 0
        Moves = []
        for x in path:
            [e,r] = x
            dx = e - preve
            dy = r - prevr
            preve = e
            prevr = r    
            Moves.append([dx,dy])
        prevdirection = [0,0]
        prevorientation = self.Orientation
        desiredorientation = 0
        moveperiod = 0
        direction = ''
        MovementInstructions = []
        for x in Moves:
            a,b = x
            changedist = ((a**2)+(b**2))**0.5  
            movetime = (changedist/self.speed)*1000
            if x == [0,10]:
                desiredorientation = 90
            elif x == [0,-10]:
                desiredorientation = 270
            elif x == [10,0]:
                desiredorientation = 0
            elif x == [-10,0]:
                desiredorientation = 180
            elif x == [10,10]:
                desiredorientation = 45
            elif x == [-10,10]:
                desiredorientation = 135
            elif x == [-10,-10]:
                desiredorientation = 225
            elif x == [10,-10]:
                desiredorientation = 315
            if x == [0,0]:
                if moveperiod != 0:
                    MovementInstructions.append(f"W:{int(round(moveperiod))}")
                moveperiod = 0
                continue
            if x == prevdirection:
                moveperiod = moveperiod + movetime
            else:     
                if moveperiod != 0:
                    MovementInstructions.append(f"W:{int(round(moveperiod))}")
                moveperiod = 0
                changeorientation = desiredorientation - prevorientation            
                turntime = abs(changeorientation/self.angularspeed)*1000
                if changeorientation < 0:
                    direction = 'D'
                elif changeorientation > 0:
                    direction = 'A'
                if turntime != 0:
                    MovementInstructions.append(f"{direction}:{int(round(turntime))}")
                moveperiod = moveperiod + movetime
            prevdirection = x
            prevorientation = desiredorientation
        MovementInstructions.append(f"W:{int(round(moveperiod))}")
        print(MovementInstructions)
        return MovementInstructions

#Function for clearing screen
def cls():
    os.system('cls' if os.name=='nt' else 'clear')
#Menu Function for running in a thread
def Menu(Robot, Robots):
    Select = input(MenuString)
    while True:
        if Select == 'Menu':
            cls()
            Select = input(MenuString)
        elif Select == '1':
            newname = input("Enter Name: ")
            newport = input("Enter Port: ")
            Robot(newname, port = newport)
            Select = input()
        elif Select == '2':
            sel = input("Select Robot: ")
            cmd = input("Enter Command: ")
            while cmd != "Exit":
                if sel in Robots:
                    try:
                        Robots[sel].command(cmd)
                        print(f"Command Sent to {sel}")
                    except:
                        print("Failed")
                elif sel == "All":
                    for x in Robots:
                        try:
                            Robots[x].command(cmd)
                            print(f"Command Sent to {x}")
                        except:
                            print(f"Failed to send to {x}")
                else:
                    print("Robot not found")
                    sel = input("Select Robot: ")
                cmd = input("Enter Command: ")
            Select = input()
        elif Select == '3':
            for x in Robots:
                print(x)
            Select = input()
        elif Select =='4':
            if len(lines) > 0:
                break
            else:
               print("No DataSources Available")
               Select = input()
        elif Select == '5':
            sel = input("Select Robot: ")
            if sel in Robots:
                try:
                    Robots[sel].connect(self.channel.port, self.channel.baudrate)
                    print("Sent")
                except:
                    print("Failed")
        elif Select =='6':
            selectr = input("Select Robot: ")
            if selectr in Robots:
                Robots[selectr].Target = [round(int(input("Enter x: ")),-1),round(int(input("Enter y: ")),-1)]
                path = AStarPathfinding(Robot = Robots[selectr], Target = Robots[selectr].Target )
                Robots[selectr].CommandList = path
                Robots[selectr].Commands = Robots[selectr].PathParse(path)
            elif selectr =="Exit":
                Select = input()
            else:
                print("Robot not found")
        elif Select =='7':
            #Change this to refine readings
            for x in Robots.values():
                try:
                    x.command("X")
                except:
                    pass
            for x in ParentReadings.values():
                if x.active:
                    x.UpdatePosition()
            ConfidenceLevel = int(input("Minimum Number of Children Readings?: "))
            if ConfidenceLevel > 0:
                for x in ParentReadings.values():
                    if x.active:
                        if len(x.children) <= ConfidenceLevel:
                            x.active = False
            Select = input()
        elif Select =='8':
            SystemExit
        else:
            Select = input()
  
def animate(i): #Animation function for FuncAnimation. Simply returns tuple (iterable list) form of the lines and readings for each robot. All actual updating of the line data is done as a class functione 
    if Robots:
        robotpos = np.c_[np.array([[x.x0,x.y0] for x in Robots.values()])]
        RobotPositions.set_offsets(robotpos)
        RobotContactAreas.set_offsets(robotpos)
    validreadings = [x.displayedposition for x in ParentReadings.values() if x.active] 
    readings = np.c_[np.array(validreadings)] #Converts the Updated reading array for use with set_offset    
    if ShowTraverseMap:
        robotpos = np.c_[np.array(x.position for x in Robots.values())]
        activatednodes = [x.position for x in PathfindingNodes.values() if x.available]
        anodes = np.c_[np.array(activatednodes)]
        AvailableArea.set_offsets(anodes)
        if readings.any():            
            deactivatednodes = [x.position for x in PathfindingNodes.values() if not x.available]
            dnodes = np.c_[np.array(deactivatednodes)]
            UnavailableArea.set_offsets(dnodes)
            ConfirmedRobotReadings.set_offsets(readings)        
            return tuple(lines3)
    if readings.any():
        ConfirmedRobotReadings.set_offsets(readings)        
        return tuple(lines2)
    else:
        return tuple(lines)
cls()
input(f"""Joshua Forrest 3rd Year Mechatronic Engineering Project
Press Enter to begin...""")
cls()
CommandMenu = threading.Thread(target = Menu, args = (Robot, Robots))
CommandMenu.start()
ani = animation.FuncAnimation(fig, animate, init_func=init, fargs=(),interval = 50, blit = True)
plt.show()