import customtkinter
import SERIAL_SENDER as mySer
import COMPUTER_VISION_MODULE as myCV
import serial
from serial.serialutil import SerialException
from SERIAL_SENDER import Arduino
import time
import cv2
from PIL import Image, ImageTk

import FORWARD_KINEMATICS_MODULE as FK
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import INVERSE_KINEMATICS_MODULE as IK
import math

#TODO: ADD TRAINING MODULE
#TODO: INTERFACE HELL



#SAME AS ARDUINO MAP() FUNCTION
def changeBasis(value, fromLow, fromHigh, toLow, toHigh):
    try:return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow
    except ZeroDivisionError as ZERO:
        print(ZERO)
        return 0


class RobotControlGUI:

    def __init__(self, master, joint_limits, demo, COMS, BAUD, init_angles):

        logo1 = 'VIMA_LOGO.png'
        self.themeColor = "darkorange"
        self.themeColor2 = "darkred"

        #=======================================================  OBJECT CONSTRUCTOR
        self.joint_limits = joint_limits    # (5, 1) indexed from 0: Describes (MIN, MAX) joint limits
        self.demo = demo                    # BOOLEAN for if in demo mode or not, toggles serial
        self.name = "VIMA"
        self.init_angles = init_angles
        ik_limits = [[-200, 200], [-200, 200], [-200, 200], [-6, 6], [-6, 6], [-6, 6]]

        #for serial stuff
        self.commsList = COMS
        self.BAUD = BAUD
        self.start_time = time.time()
        self.shaken_list = [False, False, False]




        #======================================================== S E T U P  CUSTOM TKINTER
        self.master = master
        self.master.title(f"{self.name} CONTROL GUI")
        #customtkinter.set_default_color_theme("red")
        # configure grid layout (4x4)
        self.master.geometry(f"{1100}x{580}")
        self.master.grid_columnconfigure(1, weight=1)
        self.master.grid_columnconfigure((2, 3), weight=0)
        self.master.grid_rowconfigure((0, 1, 2), weight=1)
        my_font = ("Comic Sans MS", 24, "bold")
        self.title_label = customtkinter.CTkLabel(self.master, text= "OpenVisionMimicArm",
                                                  fg_color=self.themeColor, text_color="white",width=300, height= 55,
                                                  corner_radius=8,font=my_font).place(x=700, y= 25)



        #======== SETUP SERIAL FOR ALL COMMS IN COMMS_LIST
        self.serialList = [self.serial_constructor(COM) for COM in self.commsList]
        self.MytestSerial = self.serialList[0] #Serial object for first serial coms
        self.demo = demo

        #========================================================= COMPUTER VISION
        self.cap = cv2.VideoCapture(0)
        self.camera_label = customtkinter.CTkLabel(self.master, width=640, height=480, corner_radius=8, text="")
        self.camera_label.place(x = 50, y = 0)
        self.vision = myCV.ComputerVisionBody(False) #Kalman filter bool as args**


        #========================================================  SLIDERS for Forward Kinematics

        #init frame for slider placement
        self.slider_frame = customtkinter.CTkFrame(self.master)
        #self.slider_frame.pack()
        self.slider_frame.grid(row=0, column=0, rowspan=4, sticky="nsew")
        #create a label for the frame
        self.slider_label = customtkinter.CTkLabel(self.slider_frame, text="Forward Kinematics", font=("Figtree", 15, "bold"), fg_color=self.themeColor, corner_radius=8)
        self.slider_label.place(x=25, y = 20)
        fk_slider_names = ['Base','Shoulder','Elbow','Differential_L','Differential_R','Yaw']
        #SETUP SLIDERS AND LABELS
        self.fk_sliders ,self.labelEachSlider = self.slider_constructor(self.slider_frame, self.joint_limits, fk_slider_names, 70)


        # =====INIT KINEMATICS SOLVERS
        self.Fk = FK.ForwardKinematics([-200, 200])
        self.Ik = IK.InverseKinematics([ik_limits])


        #============================================================== URDF FORWARD KINEMATICS PLOT

        self.ax, self.fig = self.Fk.ax, self.Fk.fig

        #self.fig.set_figwidth(5*2)
        #self.fig.set_figheight(4*2)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.master)
        self.robot_plot = self.canvas.get_tk_widget()
        self.robot_plot.configure(width=500*1.5, height=380*1.5)
        self.robot_plot.place(x=1100, y = 150)
        #self.Fk.main(init_angles)
        self.animate(init_angles)


        #============================================================ INVERSE KINEMATICS SLIDERS

        #Define frame to put manual controls on
        self.ik_frame = customtkinter.CTkFrame(self.master)
        self.ik_frame.grid(row=0, column = 2, rowspan=4, sticky="nsew")
        self.coordinate_bounds = [[20, ik_limits[i]] for i in range(len(ik_limits))]

        #label control frame
        self.ik_label = customtkinter.CTkLabel(self.ik_frame, text="Inverse Kinematics", font=("Figtree", 15, "bold"), fg_color=self.themeColor, corner_radius=8)
        self.ik_label.place(x = 25, y = 20)
        ik_slider_names = ['X', 'Y', 'Z', 'ALPHA', 'BETA', 'GAMMA']
        #init sliders for inverse kinematics
        self.ik_sliders, self.ik_labels =  self.slider_constructor(self.ik_frame, ik_limits, ik_slider_names, 50)

        #add logo
        self.logo_label = customtkinter.CTkLabel(self.ik_frame, text=None, fg_color=self.themeColor, corner_radius=8, height=150)
        self.logo_label.place(x=30, y=630)
        logo_img = cv2.imread(logo1)
        logo_img = cv2.resize(logo_img, (int(logo_img.shape[0] / 8), int(logo_img.shape[1] / 8)))
        self.process_image(logo_img, self.logo_label, logo_img.shape)


        #============================================================ CONTROL ACTIVATION SWITCHES

        # label control frame
        self.control_label = customtkinter.CTkLabel(self.master, text="Choose Control",font=("Figtree", 15, "bold")).place(x=220, y=370)

        # Forward Kinematics Activation Switch
        self.fk_switch_var = customtkinter.StringVar(value="on")
        self.fk_switch = customtkinter.CTkSwitch(self.master, text="Forward Kinematics", variable=self.fk_switch_var,
                                           onvalue="off", offvalue="on", progress_color= self.themeColor, button_color= "black")
        self.fk_switch.place(x=220, y = 400)



        # MIMIC Computer Vision Activation Switch
        self.mimic_switch_var = customtkinter.StringVar(value="on")
        self.mimic_switch = customtkinter.CTkSwitch(self.master, text="MIMIC Dynamics", variable=self.mimic_switch_var,
                                                 onvalue="off", offvalue="on", progress_color=self.themeColor,
                                                 button_color="black")
        self.mimic_switch.place(x=220, y=430)


        #Inverse Kinematcis Activation Switch
        self.ik_switch_var = customtkinter.StringVar(value="on")
        self.ik_switch = customtkinter.CTkSwitch(self.master, text="Inverse Kinematics", variable=self.ik_switch_var,
                                                    onvalue="off", offvalue="on", progress_color=self.themeColor,
                                                    button_color="black")
        self.ik_switch.place(x=220, y=460)

        #SWITCH THAT SENDS HOME WHEN ON
        self.home_switch_var = customtkinter.StringVar(value="on")
        self.home_switch = customtkinter.CTkSwitch(self.master, text="HOME COORDINATES", variable=self.home_switch_var,
                                                    onvalue="off", offvalue="on", progress_color=self.themeColor,
                                                    button_color="black")
        self.home_switch.place(x=220, y=490)


        #TODO, PUT TRAINER IN MENU THING, ADD COMPUTER VISION DETECTOR, MAKE TRAINER, DESIGN TRAINER
        #================================= TRAINER STORAGE
        trainer_size = 300
        self.trainer_frame = customtkinter.CTkFrame(self.master, fg_color=self.themeColor, height=trainer_size, width=trainer_size + 100)
        self.trainer_frame.place(x = 250, y = 600)
        my_font2 = ('Figtree', 24, "bold")

        self.trainer_title_frame = customtkinter.CTkFrame(self.trainer_frame, fg_color="lightgrey", width=trainer_size + 100, height= 50)
        self.trainer_title_frame.place(x=0, y=15)

        self.trainer_label = customtkinter.CTkLabel(self.trainer_title_frame, text = "Train Movements",font=my_font2, text_color="black").place(x=0, y = 10)
        # SWITCH for Training Robot MOVEMENTS
        self.train_switch_var = customtkinter.StringVar(value="on")
        self.train_switch = customtkinter.CTkSwitch(self.trainer_title_frame, text="Toggle Trainer", variable=self.train_switch_var,
                                                    onvalue="off", offvalue="on", progress_color=self.themeColor,
                                                    button_color="black", text_color="black", font=("Figtree", 15, 'bold'))
        self.train_switch.place(x=200, y=10)

        #WIDGET FOR DISPLAYING COORDIANTES
        self.angles_label = customtkinter.CTkLabel(self.trainer_frame, text = str(init_angles) , font = my_font, fg_color="grey",
                                                   corner_radius=8)
        self.angles_label.place(x=10, y = 250)








        #========================================================= HELPER WIDGETS:   CONNECTIVITY STATUS LABELS AND HANDSHAKE BUTTON
        # HANDSHAKE BUTTON
        self.handshakeButton = customtkinter.CTkButton(self.slider_frame, text= f"Initialize {self.name}", command=self.handshake_all, fg_color= self.themeColor, text_color="black",
                                                       font=("Figtree", 15, 'bold'))
        self.handshakeButton.place(x = 20, y=500)

        #SERIAL CONNECITON LABELS
        self.device_names = ['SLAVE1', 'SLAVE2', 'BLUETOOTH']
        self.device_frame = customtkinter.CTkFrame(self.master, width=310, height=60 )
        self.device_frame.place(x=225, y=25)
        self.device_label_list = self.device_contstructor()
        nonList = [device_label.place(x = 10 + 100*i, y = 20) for i, device_label in enumerate(self.device_label_list)]





        #SWITCH TO HOLD CURRENT POSITION

        #call main loop
        self.main_loop()


    #===================================================== CONSTRUCTOR FUNCTIONS
    def serial_constructor(self, COM):
        '''Function that holds serial constructor, prevents clutter in contrcutor with root'''
        try:
            #INIT SERIAL COMMUNICATIONS
            MySerial = Arduino(str(COM), int(self.BAUD))
            #if MySerial is not None:
            self.demo = False
            return MySerial

        except SerialException as SE:
            print(SE, "keep running for now, but no arudino connected")
            self.demo = True
            return None #
    #Used to define a attribute list of serial comms

    def device_contstructor(self):

        device_label_list = []

        for i in range(len(self.shaken_list)):
            device_label = customtkinter.CTkLabel(self.device_frame, fg_color=self.themeColor , text=f"{self.device_names[i]}",
                                                       text_color="white", corner_radius=8)
            #device_label.place(x=310 + 100 * i, y=30)
            device_label_list.append(device_label)

        return device_label_list
    #Used to define a attribute list of device labels


    def slider_constructor(self, frame, slider_limits, slider_names, SPACING):

        my_sliders = []
        labelEachSlider = []#70

        # for each slider name, create and place a slider with its cooresponding label
        for i, name in enumerate(slider_names):
            # init slider
            slider = customtkinter.CTkSlider(frame, button_color=self.themeColor, from_=slider_limits[i][0],
                                             to=slider_limits[i][1])
            # place sliders
            slider.place(x=0, y=120 + SPACING * i)
            my_sliders.append(slider)

            # init cooresponding labels
            label = customtkinter.CTkLabel(frame, text=name)
            # place labels
            label.place(x=0, y=80 + SPACING * i)
            labelEachSlider.append(label)

        return my_sliders, labelEachSlider
    #returns sliders and slider labels for forward kinematics and inversekinematics






    '''=============================================  START  MAIN LOOP FOR WHOLE SCRIPT'''
    def main_loop(self):

        #SET DEFAULTED INICIAL CONDITION
        requestList = self.init_angles

        try:
            #Call computer vision: Get image, Control Booleans, and list of angles
            cvThetaList, personBool, frame, shoulderBool, ret, ik_coords = myCV.inLoop_vision_body(self.vision)
            print(ik_coords)

            #Update image displayed
            self.process_image(frame, self.camera_label, (640/2, 480/2))

            #SHOULDER
            #cvThetaList ranges [13, 163] where 13 is arm all the way above the head, 163 is hand beside hip
            cvThetaList= list(cvThetaList)
            cvThetaList[1] = changeBasis(cvThetaList[1], 13, 163, self.joint_limits[1][0], self.joint_limits[1][1])




            #print(self.fk_switch.get(), self.ik_switch.get(), self.mimic_switch.get(), self.train_switch.get())

            #====  C O N T R O L S ====



            #IF FORWARD KINEMATICS SWITCH IS ON
            if not self.fk_switch.get() == "on":
                requestList = self.activate_fk()

            # IF INVERSE KINEMATICS SWITCH IS ON
            if not self.ik_switch.get() == "on":
                requestList = self.activate_ik()
                requestList = [(180/math.pi)*value for value in requestList]

            # IF MIMIC Dynamics SWITCH IS ON
            if not self.mimic_switch.get() == "on":
                if personBool and shoulderBool:
                    gyroControlList = [0,0,0]  #TODO: GET GYRO CONTROL FROM ARDUINO
                    requestList = list(cvThetaList) + gyroControlList


            #====  C O N T R O L S ====

            # IF TRAINING  SWITCH IS ON
            if not self.train_switch.get() == "on":
                self.start_training(requestList)



            # Convert to int, change of basis
            requestList = [int(value) for value in requestList]
            print(requestList)


            #SEND THROUGH SERIAL
            for i, i_serial in enumerate(self.serialList):
                try:
                    if self.shaken_list[i]:
                        i_serial.inLoopSender(requestList, True)
                    else: print("not shaken", zip(self.shaken_list, self.device_names))
                except IndexError as IE:
                    print("INDEX ERROR DURING SENDING", IE)


            #SHOW ON URDF PLOTTER
            self.animate(requestList)
            self.angles_label.configure(text=str(requestList))

            end_time = time.time()

            print("TIME:", self.start_time - end_time)
            self.start_time = end_time




        except AttributeError as error:
            print(error)
        except TypeError as error:
            print(error)
        except ZeroDivisionError as error:
            print(error)



        #run whole loop
        self.master.after(1, self.main_loop)

    '''=============================================   END MAIN LOOP FOR WHOLE SCRIPT'''






    #====== F U N C T I O N S ======

    def home_sliders(self):
        noneList = [slider.setvar(self.init_angles[i]) for i,slider in enumerate(self.fk_sliders)]
        noneList = [slider.setvar(self.init_angles[i]) for i,slider in enumerate(self.ik_sliders)]


    def handshake_all(self):
        '''This function is called by a button on the GUI. When pressed, if the GUI is not in demo mode,
                    the function will establish a serial communications handshake with each slave control module.

            ALSO established the bluetooth data stream from arduino running extended kalman filter

                SLAVE MODULES:
                        CTRL1: VIA SERIAL USB: Controls Base Shoulder an Elbow
                        CTRL2: VIA SERIAL USB: Controls Yaw, Differential Left, and Differential Right
                        CTRL3: VIA BLUETOOTH : GET KALMAN FILTERED DATA FROM ARDUINO '''
        #IF Process isnt in demo mode
        print(f"Demo Bool: {self.demo}, Serial List: {self.serialList}")
        if not self.demo:
            #Handshake all serial connecitons and return a list of which ones are connected
            self.shaken_list = [self.handshake(MySerial) for MySerial in self.serialList]
        else:print(f"No INIT IN PLACE, Connect {self.name} and Restart program")

        #UPDATE COLORED labels to show if connected or not
        self.update_connection_status()
        print("SHAKING DONE", self.shaken_list)
    #Establishes handshaken state for all serials in self.serialList
    def handshake(self, MySerial):
        '''Establishes reliable serial communications via a handshake approach for one serial object'''


        # SHAKE HANDS UNTIL ARUDINO IS READY
        try:
            ardu_string = str(MySerial.get_data())
            while not "READY" in ardu_string:
                MySerial.send_data("SHAKING", True)
                print("SHAKING", ardu_string, MySerial)
                time.sleep(1)
                ardu_string = str(MySerial.get_data())

            if MySerial is not None: return True
            else: return False

        except AttributeError as AE:
            print(AE, "probrally no serial device")
            return False
    #Used in self.handshake_all


    '''======   C O N T R O L S  ======'''
    def activate_ik(self):
        print("INVERSE KINEMATICS ACTIVATED")
        requestCoords = self.get_slider_list(self.ik_sliders)

        #requestCoords = [self.coordinate_bounds[i][0] for i,coord in enumerate(requestCoords) if coord< self.coordinate_bounds[i][0]]
        #requestCoords = [self.coordinate_bounds[i][1] for i, coord in enumerate(requestCoords) if coord>self.coordinate_bounds[i][1]]

        requestList = self.Ik.main(requestCoords)

        return requestList

    def activate_fk(self):
        print("FORWARD KINEMATICS ACTIVATED")
        requestList = self.get_slider_list(self.fk_sliders)
        return requestList


    def start_training(self):
        print("GATHERING TRAINING DATA")

        return self.init_angles

    def send_home(self):
        return self.init_angles

    '''======   C O N T R O L S  ======'''




    def get_slider_list(self, sliders):
        slider_values = [slider.get() for slider in sliders]
        return slider_values
    #Returns the values from the slider List inputed

    def animate(self, requestList):

        #make update to the robot plot
        self.Fk.main(requestList)
    #Animates the requestList given on URDF model of robot

    def update_connection_status(self):
        for i,label in enumerate(self.device_label_list):
            try:
                if self.shaken_list[i]:
                    print(f"Serial Connection made with {self.device_names[i]}")
                    label.configure(fg_color = "darkgreen")
            except IndexError as IE:
                print(IE, "issue with index raised")
    #Called after handshake to update serial connection status

    def process_image(self, frame, label, size):

        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(image)
        photo = customtkinter.CTkImage(image, size= size)
        label.configure(image=photo)
        label.image = photo

        #self.master.after(10, self.display_loop)
    #Called in main_loop to update and maintain

    def run(self):
        self.master.mainloop()
    #INHERITS RUNNING MAINLOOP











joint_limits2 = [[-180, 180],
                [-90, 90],
        
                [-120, 120],
                [-180, 180],
                [-180, 180],
                [-90, 90]]

joint_limits = [[0, 360],
                [-90, 90],
                [0, 180],
                [-180, 180],
                [-180, 180],
                [-90, 90]]



# Create an instance of the RobotControlGUI class and run the GUI
root = customtkinter.CTk()
#root.attributes("-fullscreen", True)

COMS = ["COM3", "COM8"]
init_angles = [0,-17,109,0,0,0]


try:
    # root, joint_limits, demo_mode_bool, serialARGS
    app = RobotControlGUI(root, joint_limits, False, COMS, 115200, init_angles)
    app.run()
except KeyboardInterrupt as KE:
    print("TERMINATING SCRIPT")
    #Send home values