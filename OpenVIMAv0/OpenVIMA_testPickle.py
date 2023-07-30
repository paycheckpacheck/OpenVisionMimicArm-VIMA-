import customtkinter
import SERIAL_SENDER as mySer
import COMPUTER_VISION_MODULE as myCV
import serial
from serial.serialutil import SerialException
from SERIAL_SENDER import Arduino
import time
import cv2
from PIL import Image, ImageTk
import numpy as np
import FORWARD_KINEMATICS_MODULE as FK
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import INVERSE_KINEMATICS_MODULE as IK
import math
import pickle
import TRAINER_MODULE as trainer


# TODO: ADD TRAINING MODULE
# TODO: INTERFACE HELL
#TODO: Debuf interface within gui
#TODO: Display arduino message in GUI

# SAME AS ARDUINO MAP() FUNCTION
def changeBasis(value, fromLow, fromHigh, toLow, toHigh):
    try:
        return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow
    except ZeroDivisionError as ZERO:
        print(ZERO)
        return 0
hover = "#CF4420"



class ScrollableLabelButtonFrame(customtkinter.CTkScrollableFrame):
    def __init__(self, master, command=None,  **kwargs):
        super().__init__(master, **kwargs)
        self.grid_columnconfigure(0, weight=1)

        self.labelfont = ("Figtree", 15, "bold")


        self.command = command
        self.radiobutton_variable = customtkinter.StringVar()
        self.label_list = []
        self.button_list = []

    def add_item(self, item, button_text, image=None):
        label = customtkinter.CTkLabel(self, text=item, image=image, compound="left", padx=5, anchor="w",
                                       font=self.labelfont,text_color="black")
        button = customtkinter.CTkButton(self, text=button_text, hover_color=hover,
                                         width=100, height=24, fg_color="darkred", text_color="white")
        if self.command is not None:
            button.configure(command=lambda: self.command(item))
        label.grid(row=len(self.label_list), column=0, pady=(0, 10), sticky="w")
        button.grid(row=len(self.button_list), column=1, pady=(0, 10), padx=5)
        self.label_list.append(label)
        self.button_list.append(button)
    #Add item to tkinter scroller

    def remove_item(self, item):
        for label, button in zip(self.label_list, self.button_list):
            if item == label.cget("text"):
                label.destroy()
                button.destroy()
                self.label_list.remove(label)
                self.button_list.remove(button)
                return
    #Remove Item from tkinter widget

class RobotControlGUI:

    def __init__(self, master, joint_limits, demo, COMS, BAUD, init_angles, init_coords, folder_path):


        vt_maroon = "#861F41"

        logo1 = 'VIMA_LOGO.png'
        self.themeColor = vt_maroon#"darkred"
        self.themeColor2 = vt_maroon#"darkred"
        my_text_color = "white"
        self.btn_font = ("Figtree", 15, "bold")
        self.label_font = ("Figtree", 12, "bold")
        # used for label coloring, not exclusigve to the sliders
        self.slider_fg_color = "lightgrey"
        self.slider_txt_color  = "maroon"
        button_border_color = "black"
        switch_color = "grey"



        # =======================================================  OBJECT CONSTRUCTOR
        self.joint_limits = joint_limits  # (5, 1) indexed from 0: Describes (MIN, MAX) joint limits
        self.demo = demo  # BOOLEAN for if in demo mode or not, toggles serial
        self.name = "VIMA"
        self.init_angles = init_angles
        self.init_coords = init_coords
        ik_limits = [[-200, 200], [-200, 200], [-200, 200], [-6, 6], [-6, 6], [-6, 6]]

        # for serial stuff
        self.commsList = COMS
        self.BAUD = BAUD
        self.start_time = time.time()
        self.shaken_list = [False, False, False]

        # ======================================================== S E T U P  CUSTOM TKINTER
        self.master = master
        self.master.title(f"{self.name} CONTROL GUI")
        # customtkinter.set_default_color_theme("red")
        # configure grid layout (4x4)
        self.master.geometry(f"{1100}x{580}")
        #self.master.grid_columnconfigure(8, weight=1)
        span = 1
        self.master.grid_columnconfigure(span, weight=1)
        self.master.grid_rowconfigure(span, weight=1)
        my_font = ("Comic Sans MS", 24, "bold")
        '''self.title_label = customtkinter.CTkLabel(self.master, text="OpenVisionMimicArm",
                                                  fg_color=self.themeColor, text_color="white", width=300, height=55,
                                                  corner_radius=8, font=my_font).place(x=700, y=25)'''

        # ======== SETUP SERIAL FOR ALL COMMS IN COMMS_LIST
        self.serialList = [self.serial_constructor(COM) for COM in self.commsList]
        self.MytestSerial = self.serialList[0]  # Serial object for first serial coms
        self.demo = demo

        # ========================================================= COMPUTER VISION
        self.cap = cv2.VideoCapture(0)
        self.camera_label = customtkinter.CTkLabel(self.master, width=640, height=480, corner_radius=8, text="")
        self.camera_label.place(x=600, y=500)
        self.vision = myCV.ComputerVisionBody(False)  # Kalman filter bool as args**

        # ========================================================  SLIDERS for Forward Kinematics

        # init frame for slider placement
        self.slider_frame = customtkinter.CTkFrame(self.master)
        # self.slider_frame.pack()
        self.slider_frame.grid(row=0, column=0, rowspan=4, sticky="nsew")
        # create a label for the frame
        self.slider_btn = customtkinter.CTkButton(self.slider_frame, text="Forward Kinematics",hover_color=hover,
                                                   font=("Figtree", 15, "bold"), fg_color=self.themeColor,
                                                   corner_radius=8, command=self.home_sliders, text_color=my_text_color)
        self.slider_btn.place(x=25, y=20)
        fk_slider_names = ['Base', 'Shoulder', 'Elbow', 'Differential_L', 'Differential_R', 'Yaw']
        # SETUP SLIDERS AND LABELS
        self.fk_sliders, self.labelEachSlider = self.slider_constructor(self.slider_frame, self.joint_limits,
                                                                        fk_slider_names, 115)

        # =====INIT KINEMATICS SOLVERS
        self.Fk = FK.ForwardKinematics([-200, 200])
        self.Ik = IK.InverseKinematics([ik_limits])

        # ============================================================== URDF FORWARD KINEMATICS PLOT

        self.ax, self.fig = self.Fk.ax, self.Fk.fig

        #self.fig.set_figwidth(5*2)
        #self.fig.set_figheight(4*2)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.master)
        self.robot_plot = self.canvas.get_tk_widget()
        self.robot_plot.configure(width=640*2.1, height=480*1.4, selectforeground='maroon')
        #self.canvas.get_renderer()
        self.robot_plot.place(x=850, y=170)
        # self.Fk.main(init_angles)
        self.animate(init_angles)

        # ============================================================ INVERSE KINEMATICS SLIDERS

        # Define frame to put manual controls on
        self.ik_frame = customtkinter.CTkFrame(self.master)
        self.ik_frame.grid(row=0, column=3, rowspan=4, sticky="nsew")
        self.coordinate_bounds = [[20, ik_limits[i]] for i in range(len(ik_limits))]

        # label control frame
        self.ik_btn = customtkinter.CTkButton(self.ik_frame, text="Inverse Kinematics", font=("Figtree", 15, "bold"), hover_color=hover,
                                               fg_color=self.themeColor, corner_radius=8, text_color= my_text_color, command=self.home_sliders)
        self.ik_btn.place(x=25, y=20)
        ik_slider_names = ['X', 'Y', 'Z', 'ALPHA', 'BETA', 'GAMMA']
        # init sliders for inverse kinematics
        self.ik_sliders, self.ik_labels = self.slider_constructor(self.ik_frame, ik_limits, ik_slider_names, 115)

        # add logo
        self.logo_label = customtkinter.CTkLabel(self.ik_frame, text=None, fg_color=self.themeColor, corner_radius=8,
                                                 height=150)
        self.logo_label.place(x=30, y=770)
        logo_img = cv2.imread(logo1)
        logo_img = cv2.resize(logo_img, (int(logo_img.shape[0] / 8), int(logo_img.shape[1] / 8)))
        self.process_image(logo_img, self.logo_label, logo_img.shape)

        # ============================================================ CONTROL ACTIVATION SWITCHES

        # label control frame

        self.ctrl_frame = customtkinter.CTkFrame(self.slider_frame, fg_color= self.themeColor, corner_radius=8, height= 150, width = 180)
        self.ctrl_frame.place(x=10, y=750)
        self.control_label = customtkinter.CTkLabel(self.ctrl_frame, text="Choose Control",
                                                    font=self.btn_font, fg_color= self.slider_fg_color,
                                                    text_color= self.slider_txt_color, corner_radius=8).place(x=20, y=8)

        # Forward Kinematics Activation Switch
        self.fk_switch_var = customtkinter.StringVar(value="on")
        self.fk_switch = customtkinter.CTkSwitch(self.ctrl_frame, text="Forward Kinematics", variable=self.fk_switch_var,
                                                 onvalue="off", offvalue="on",
                                                 progress_color=self.slider_txt_color,button_color=switch_color, font= self.label_font, border_color=button_border_color, button_hover_color = hover)
        self.fk_switch.place(x=10, y=40)

        # MIMIC Computer Vision Activation Switch
        self.mimic_switch_var = customtkinter.StringVar(value="on")
        self.mimic_switch = customtkinter.CTkSwitch(self.ctrl_frame, text="MIMIC Dynamics", variable=self.mimic_switch_var,
                                                    onvalue="off", offvalue="on",
                                                    progress_color=self.slider_txt_color, button_color=switch_color,
                                                    font=self.label_font, border_color=button_border_color,
                                                    button_hover_color=hover)

        self.mimic_switch.place(x=10, y=65)

        # Inverse Kinematcis Activation Switch
        self.ik_switch_var = customtkinter.StringVar(value="on")
        self.ik_switch = customtkinter.CTkSwitch(self.ctrl_frame, text="Inverse Kinematics", variable=self.ik_switch_var,
                                                 onvalue="off", offvalue="on",
                                                 progress_color=self.slider_txt_color, button_color=switch_color,
                                                 font=self.label_font, border_color=button_border_color,
                                                 button_hover_color=hover)

        self.ik_switch.place(x=10, y=90)

        # Inverse Kinematcis Hand tracking Activation Switch
        self.ik_coord_switch_var = customtkinter.StringVar(value="on")
        self.ik_coord_switch = customtkinter.CTkSwitch(self.ctrl_frame, text="Hand Tracking",
                                                 variable=self.ik_coord_switch_var,
                                                 onvalue="off", offvalue="on",
                                                 progress_color=self.slider_txt_color, button_color=switch_color,
                                                 font=self.label_font, border_color=button_border_color,
                                                 button_hover_color=hover)

        self.ik_coord_switch.place(x=10, y=115)





        # ================================= TRAINER STORAGE
        self.myTrainer = trainer.TrainerWidget(self.master, folder_path, self.init_angles, [215,120])





        # ========================================================= HELPER WIDGETS:   CONNECTIVITY STATUS LABELS AND HANDSHAKE BUTTON


        # SERIAL CONNECITON LABELS
        self.device_names = ['SLAVE1', 'SLAVE2', 'BLUETOOTH']
        self.device_frame = customtkinter.CTkFrame(self.master, width=450, height=60)
        self.device_frame.place(x=225, y=25)
        self.device_label_list = self.device_contstructor()

        # HANDSHAKE BUTTON
        self.handshakeButton = customtkinter.CTkButton(self.device_frame, text=f"Initialize {self.name}",
                                                       command=self.handshake_all, fg_color=self.themeColor, hover_color=hover,
                                                       text_color=my_text_color,
                                                       font=self.btn_font)
        #place
        nonList = [device_label.place(x=180 + 80 * i, y=20) for i, device_label in enumerate(self.device_label_list)]
        self.handshakeButton.place(x=20, y=20)

        # WIDGET FOR DISPLAYING ANGLES
        self.angles_label = customtkinter.CTkLabel(self.master, text=f"FK:{str(init_angles)}", font=my_font,
                                                   fg_color="grey", height= 60,
                                                   corner_radius=8 , text_color= self.slider_txt_color)
        self.angles_label.place(x=650, y=25)

        self.ik_coords = init_coords
        # WIDGET FOR Coordinates
        self.ik_coord_label = customtkinter.CTkLabel(self.master, text=f"IK:{str(self.ik_coords)}", font=my_font,
                                                   fg_color="grey", height=60,
                                                   corner_radius=8, text_color=self.slider_txt_color)
        self.ik_coord_label.place(x=950, y=25)



        # call main loop
        self.main_loop()

    # ===================================================== CONSTRUCTOR FUNCTIONS
    def serial_constructor(self, COM):
        '''Function that holds serial constructor, prevents clutter in contrcutor with root'''
        try:
            # INIT SERIAL COMMUNICATIONS
            MySerial = Arduino(str(COM), int(self.BAUD))
            # if MySerial is not None:
            self.demo = False
            return MySerial

        except SerialException as SE:
            print(SE, "keep running for now, but no arudino connected")
            self.demo = True
            return None  #

    # Used to define a attribute list of serial comms

    def device_contstructor(self):

        device_label_list = []

        for i in range(len(self.shaken_list)):
            device_label = customtkinter.CTkLabel(self.device_frame, fg_color=self.slider_fg_color,
                                                  text=f"{self.device_names[i]}",
                                                  text_color=self.slider_txt_color, corner_radius=8, font=self.label_font)
            # device_label.place(x=310 + 100 * i, y=30)
            device_label_list.append(device_label)

        return device_label_list

    # Used to define a attribute list of device labels

    def slider_constructor(self, frame, slider_limits, slider_names, SPACING):

        my_sliders = []
        labelEachSlider = []  # 70

        # for each slider name, create and place a slider with its cooresponding label
        for i, name in enumerate(slider_names):
            # init slider
            slider = customtkinter.CTkSlider(frame, button_color=self.themeColor, from_=slider_limits[i][0],
                                             to=slider_limits[i][1], button_hover_color=hover)
            # place sliders
            slider.place(x=0, y=120 + SPACING * i)
            my_sliders.append(slider)

            # init cooresponding labels
            label = customtkinter.CTkLabel(frame, text=name, font= self.label_font, fg_color = self.slider_fg_color,
                                           text_color=self.slider_txt_color, corner_radius=8, width=50)
            # place labels
            label.place(x=20, y=80 + SPACING * i)
            labelEachSlider.append(label)

        return my_sliders, labelEachSlider

    # returns sliders and slider labels for forward kinematics and inversekinematics

    '''=============================================  START  MAIN LOOP FOR WHOLE SCRIPT'''

    def main_loop(self):

        # SET DEFAULTED INICIAL CONDITION
        requestList = self.init_angles

        try:
            # Call computer vision: Get image, Control Booleans, and list of angles
            cvThetaList, personBool, frame, shoulderBool, ret, self.ik_coords = myCV.inLoop_vision_body(self.vision)


            # Update image displayed
            self.process_image(frame, self.camera_label, (640*.8 , 480*.7))

            # SHOULDER
            # cvThetaList ranges [13, 163] where 13 is arm all the way above the head, 163 is hand beside hip
            cvThetaList = list(cvThetaList)
            cvThetaList[1] = changeBasis(cvThetaList[1], 13, 163, self.joint_limits[1][0], self.joint_limits[1][1])

            # print(self.fk_switch.get(), self.ik_switch.get(), self.mimic_switch.get(), self.train_switch.get())

            # ====  C O N T R O L S ====N: On is off, and off is on

            # IF FORWARD KINEMATICS SWITCH IS ON
            if not self.fk_switch.get() == "on":
                requestList = self.activate_fk()

            # IF INVERSE KINEMATICS SWITCH IS ON
            if not self.ik_switch.get() == "on":
                requestList = self.activate_ik()
                requestList = [(180 / math.pi) * value for value in requestList]

            # IF INVERSE KINEMATICS HAND TRACKING SWITCH IS ON
            if not self.ik_coord_switch.get() == "on":
                fingerClickBool = True #todo, makefinger bool switch
                if fingerClickBool and self.vision.area != 0:
                    requestList = self.activate_hand_tracking()
                    requestList = [(180 / math.pi) * value for value in requestList]


            # IF MIMIC Dynamics SWITCH IS ON
            if not self.mimic_switch.get() == "on":
                if personBool and shoulderBool:
                    gyroControlList = [0, 0, 0]  # TODO: GET GYRO CONTROL FROM ARDUINO
                    requestList = list(cvThetaList) + gyroControlList
            # ====  C O N T R O L S ==== end


            # IF TRAINING  SWITCH IS ON
            if not self.myTrainer.trainer_switch.get() == "on":
                self.start_training(requestList)

            #If data has been added to request_data_queue then pop off a data piece and use it
            if self.myTrainer.queue_data is not None and self.myTrainer.queue_data !=[]:
                requestList = self.activate_trainer()

            # Convert to int, change of basis if needed
            requestList = [int(value) for value in requestList]
            print(requestList)

            # SEND THROUGH SERIAL
            for i, i_serial in enumerate(self.serialList):
                try:
                    if self.shaken_list[i]:
                        i_serial.inLoopSender(requestList, True)
                    else:
                        print("not shaken", zip(self.shaken_list, self.device_names))
                except IndexError as IE:
                    print("INDEX ERROR DURING SENDING", IE)

            # SHOW ON URDF PLOTTER
            self.animate(requestList)
            self.angles_label.configure(text=str(requestList))
            self.ik_coord_label.configure(text=str(self.ik_coords))



            #Calculate loop time
            end_time = time.time()
            print("TIME:", self.start_time - end_time)
            self.start_time = end_time

        except AttributeError as error:
            print(error)
        except TypeError as error:
            print(error)
        except ZeroDivisionError as error:
            print(error)
        except IndexError as PersonMissing:
            print("Person missing", PersonMissing)

        # run whole loop
        self.master.after(1, self.main_loop)
        #self.master.update()

    '''=============================================   END MAIN LOOP FOR WHOLE SCRIPT'''

    # ====== F U N C T I O N S ======

    def home_sliders(self):
        noneList = [slider.set(self.init_angles[i]) for i, slider in enumerate(self.fk_sliders)]
        noneList = [slider.set(self.init_coords[i]) for i, slider in enumerate(self.ik_sliders)]

    def handshake_all(self):
        '''This function is called by a button on the GUI. When pressed, if the GUI is not in demo mode,
                    the function will establish a serial communications handshake with each slave control module.

            ALSO established the bluetooth data stream from arduino running extended kalman filter

                SLAVE MODULES:
                        CTRL1: VIA SERIAL USB: Controls Base Shoulder an Elbow
                        CTRL2: VIA SERIAL USB: Controls Yaw, Differential Left, and Differential Right
                        CTRL3: VIA BLUETOOTH : GET KALMAN FILTERED DATA FROM ARDUINO '''
        # IF Process isnt in demo mode
        print(f"Demo Bool: {self.demo}, Serial List: {self.serialList}")
        if not self.demo:
            # Handshake all serial connecitons and return a list of which ones are connected
            self.shaken_list = [self.handshake(MySerial) for MySerial in self.serialList]
        else:
            print(f"No INIT IN PLACE, Connect {self.name} and Restart program")

        # UPDATE COLORED labels to show if connected or not
        self.update_connection_status()
        print("SHAKING DONE", self.shaken_list)

    # Establishes handshaken state for all serials in self.serialList
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

            if MySerial is not None:
                return True
            else:
                return False

        except AttributeError as AE:
            print(AE, "probrally no serial device")
            return False

    # Used in self.handshake_all

    '''======   C O N T R O L S  ======'''

    def activate_ik(self):
        print("INVERSE KINEMATICS ACTIVATED")
        requestCoords = self.get_slider_list(self.ik_sliders)
        self.ik_coords = [int(coord) for coord in requestCoords]

        # requestCoords = [self.coordinate_bounds[i][0] for i,coord in enumerate(requestCoords) if coord< self.coordinate_bounds[i][0]]
        # requestCoords = [self.coordinate_bounds[i][1] for i, coord in enumerate(requestCoords) if coord>self.coordinate_bounds[i][1]]

        requestList = self.Ik.main(requestCoords)

        return requestList

    def activate_hand_tracking(self):

        slider_coord = self.get_slider_list(self.ik_sliders)




        self.ik_coords = [self.ik_coords[2], self.ik_coords[0], self.ik_coords[1],slider_coord[3] , slider_coord[4], slider_coord[5]]
        requestList = self.Ik.main(self.ik_coords )
        return requestList



    def activate_fk(self):
        print("FORWARD KINEMATICS ACTIVATED")
        requestList = self.get_slider_list(self.fk_sliders)
        return requestList


    def activate_trainer(self):

        request_queue_data_np = np.array(self.myTrainer.queue_data)
        link_list = self.myTrainer.queue_data

        #if dim 3, then the reuestList is from multipule queues
        if request_queue_data_np.ndim == 3:
            #Get the last appended link and use as if it where a singular link
            link_list = self.myTrainer.queue_data.pop()


        #Get request List from linked list
        requestList = link_list.pop(0)



        print(f"Trainer has been called, writing prestored movments:{requestList}")

        #If list is empty after the pop
        if len(self.myTrainer.queue_data) ==0:
            #clear list and make empty
            self.myTrainer.queue_data = []
        return requestList
    #Called in loop, if their is training data in attrib list, uses that data instead, clears the attribute when done using data


    def start_training(self, requestList):
        print("GATHERING TRAINING DATA")
        self.myTrainer.data.append(requestList)
    #Appends all new incoming data into myTrainer.data stream




    '''======   C O N T R O L S  ======'''

    def get_slider_list(self, sliders):
        slider_values = [slider.get() for slider in sliders]
        return slider_values

    # Returns the values from the slider List inputed

    def animate(self, requestList):

        # make update to the robot plot
        self.Fk.main(requestList)

    # Animates the requestList given on URDF model of robot

    def update_connection_status(self):
        for i, label in enumerate(self.device_label_list):
            try:
                if self.shaken_list[i]:
                    print(f"Serial Connection made with {self.device_names[i]}")
                    label.configure(fg_color="darkgreen")
            except IndexError as IE:
                print(IE, "issue with index raised")

    # Called after handshake to update serial connection status

    def process_image(self, frame, label, size):

        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(image)
        photo = customtkinter.CTkImage(image, size=size)
        label.configure(image=photo)
        label.image = photo

        # self.master.after(10, self.display_loop)

    # Called in main_loop to update and maintain

    def run(self):
        self.master.mainloop()
    # INHERITS RUNNING MAINLOOP


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
root.configure(bg_color = "maroon")


# root.attributes("-fullscreen", True)

customtkinter.set_appearance_mode("dark")

COMS = ["COM3", "COM8"]
init_angles = [0, -17, 109, 0, 0, 0]
init_coords = [200,0,200,0,0,0]

folder_path = r'DATA_FROM_TRAINER'

try:
    # root, joint_limits, demo_mode_bool, serialARGS
    app = RobotControlGUI(root, joint_limits, False, COMS, 115200, init_angles, init_coords, folder_path)
    app.run()
except KeyboardInterrupt as KE:
    print("TERMINATING SCRIPT")
    # Send home values