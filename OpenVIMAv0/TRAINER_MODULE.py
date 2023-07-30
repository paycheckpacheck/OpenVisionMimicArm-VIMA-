import customtkinter
import os
import pickle

hover = "#CF4420"


class ScrollableLabelButtonFrame(customtkinter.CTkScrollableFrame):
    def __init__(self, master, command=None,  **kwargs):
        super().__init__(master, **kwargs)
        self.grid_columnconfigure(0, weight=1)

        self.labelfont = ("Figtree", 12, "bold")
        self.my_txt_clr = "lightgrey"
        self.btn_font = ("Figtree", 15, "bold")



        self.command = command
        self.radiobutton_variable = customtkinter.StringVar()
        self.label_list = []
        self.button_list = []

    def add_item(self, item, button_text, image=None):
        label = customtkinter.CTkLabel(self, text=item, image=image, compound="left", padx=5, anchor="w",
                                       font=self.labelfont,text_color=self.my_txt_clr)
        button = customtkinter.CTkButton(self, text=button_text,
                                         width=100, height=24, fg_color="darkred", text_color="white", font=self.btn_font,hover_color= hover)
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





class TrainerWidget:

    def __init__(self, master, folder_path, initAngles, placecoord):

        #Configure root
        self.master = master
        #CONFIGURE POSTIIONS
        self.y_level_frames = 110
        button_Xo = 180
        button_width = 70
        spacing = 1.5
        theme_color = "maroon"
        button_yo = 40
        labelfont = ("Figtree", 12, "bold")
        buttonfont = ("Figtree", 15, "bold")
        self.label_fg = "lightgrey"
        self.label_txt = 'maroon'
        switch_color = "grey"
        button_border_color = "black"
        self.slider_fg_color = "lightgrey"
        self.slider_txt_color  = "maroon"

        instructions = ["1. Toggle the trainer on which starts the recording of data.",
                        "2.Move the robotic arm with the given controls while toggle is on.",
                        "3.When done recording turn toggle off.",
                        "4.Enter a name to name the movement you just made.",
                        "5.Click Add New Link to store the movements.",
                        "6. Press add to queue to prepare movement.",
                        "7.."]
        #Frame to keep the entire widget
        self.widget_frame = customtkinter.CTkFrame(self.master, width=350, height=760 , corner_radius=8)
        self.widget_frame.place(x = placecoord[0], y = placecoord[1])

        #Path to the folder contiaining all the pickle packages
        self.folder_path = folder_path

        #For ts
        self.initAngles = initAngles

        #Previously stored pickle packet names
        self.sheet_names = self.get_sheet_names()
        #Append sheet names to queue list
        self.sheet_names_in_queue = []

        #the attribute where training data is appended to
        self.data = []
        #The attribute where GUI reads from to start queue
        self.queue_data = []




        #Button to clear the current data within the trainer
        self.clear_data_btn = customtkinter.CTkButton(self.widget_frame, text = "Clear Current Trainer Data",
                                                      command = self.clear_trainer_data, fg_color=theme_color, font=("Figtree", 15, 'bold'), hover_color= hover).place(x=10,y=self.y_level_frames*5.8)



        # ==== SCROLLERS ====

        #SCROLLER FOR STORED MOVEMENTS
        self.new_link_frame_label = customtkinter.CTkLabel(self.widget_frame, text = "STORED LINKS", font=labelfont,
                                                           fg_color=self.label_fg, text_color=self.label_txt, corner_radius=8).place(x=15, y = self.y_level_frames - 35)
        self.movement_scroller = ScrollableLabelButtonFrame(self.widget_frame, width = 300, corner_radius=8, command=self.addLink2Queue)
        self.movement_scroller.place(x = 10, y = self.y_level_frames)
        #Add the inicial stored movements from previous use cases
        self.restore_previous_links()

        #Scrollable frame with queue
        self.new_link_frame_label = customtkinter.CTkLabel(self.widget_frame, text = "MOVMENT QUEUE", font=labelfont,
                                                           fg_color=self.label_fg, text_color=self.label_txt, corner_radius=8).place(x=15, y = self.y_level_frames*3.5 - 35)
        self.queue_scroller = ScrollableLabelButtonFrame(self.widget_frame, width = 300, corner_radius=8, command=self.runThisLink)
        self.queue_scroller.place(x= 10, y = self.y_level_frames*3.5 )


        #==== HELPER WIDGETS ====#

        #Textbox for adding new modules
        self.textbox_label = customtkinter.CTkLabel(self.widget_frame,  text="New Link Name", font = labelfont).place(x = 20, y = button_yo-25)
        self.module_textbox = customtkinter.CTkEntry(self.widget_frame, text_color="white", fg_color=theme_color)
        self.module_textbox.place(x = 20, y = button_yo)

        #button for adding new modules
        self.add_new_movement_button = customtkinter.CTkButton(self.widget_frame,hover_color= hover, command=self.add_trained_movement,fg_color=theme_color,
                                                               width= button_width, text = "Add New Link", font=buttonfont)
        self.add_new_movement_button.place(x  = button_Xo, y = button_yo)



        #Button for starting Queue
        self.start_queue_button = customtkinter.CTkButton(self.widget_frame, fg_color=theme_color,command=self.start_queue,
                                                          width= button_width, text="Start Queue", font=buttonfont,hover_color= hover)
        self.start_queue_button.place(x= button_Xo , y = self.y_level_frames*3.5 - 35 )


        #Switch for starting training
        self.trainer_switch_var = customtkinter.StringVar(value="on")
        self.trainer_switch = customtkinter.CTkSwitch(self.widget_frame, text="Toggle Trainer", variable=self.trainer_switch_var,
                                                 onvalue="off", offvalue="on",
                                                      progress_color=self.slider_txt_color, button_color=switch_color,
                                                      font=buttonfont, border_color=button_border_color,
                                                      button_hover_color=hover)

        self.trainer_switch.place(x=10 + 2*button_width, y = self.y_level_frames - 35)


        #Button for clearing everything in queue
        self.clear_queue_button = customtkinter.CTkButton(self.widget_frame, fg_color=theme_color,command=self.clearQueue,
                                                          width= button_width, text="Clear All Links from Queue", font=buttonfont,hover_color= hover)
        self.clear_queue_button.place(x = 10,  y=self.y_level_frames*5.8 + 40)
        #for i,instruction in enumerate(instructions):
            #customtkinter.CTkLabel(self.widget_frame, text = instruction, text_color= "white", fg_color="maroon", corner_radius=8).place(x = 10, y = 20*spacing*i +self.y_level_frames*5.7)



    def get_sheet_names(self):
        try:

            pickle_packets = []
            #Look in the folder pickle packets are supoosed to be in
            for filename in os.listdir(self.folder_path):
                #Store in list
                pickle_packets.append(str(filename))
                print(filename)
            #list all pickle packet names, return names
        except FileNotFoundError as NoFileinFOlder:
            print("No file in folder currently, makining init demo file")
            new_file_path = fr"{self.folder_path}/demoFile.pkl"
            with open(new_file_path, 'wb') as f:
                pickle.dump(self.initAngles, f)
                pickle_packets = self.get_sheet_names()

        return pickle_packets
    #Return the list of pick;e packets within the self.folder_path folder
    def clear_trainer_data(self):
        self.data = []
    #makes self.data attribute empty to the call of a button, can reimplmemt with lambda

    def restore_previous_links(self):
        #for all pickle packets in sheet names list
        for link_name in self.sheet_names:
            # Make new item in Stored movments scroller with button
            self.movement_scroller.add_item(link_name.replace(".pkl", ""), "Add to Queue")

        pass
    #TODO: Add links from files, dont edit data
    def add_trained_movement(self):

        #Get name in textbox
        link_name = self.module_textbox.get()
        print(f"Adding {link_name} to Stored links")
        #link_name = fr"{link_name}.pkl"

        #Make new item in Stored movments scroller with button
        self.movement_scroller.add_item(link_name, "Add to Queue")

        #Make a new pickle packet withing data from trinaer folder names link name
        filepath = fr"DATA_FROM_TRAINER/{link_name}.pkl"
        with open(filepath, 'wb') as f:
            pickle.dump(self.data, f)

        #refresh sheet names by recalling the get sheet functuon
        self.sheet_names = self.get_sheet_names()

        #Clear all the data within the data attribute
        self.clear_trainer_data()


        #TODO, MAKE NEW SHEET IN WORKBOOK, ADD TRAINED DATA
    #Adds a new link to the stored links scorller, also makes new pickle packet if doesnt exisit
    def addLink2Queue(self, item):
        print(f"Adding {item} to Queue")
        item = fr"{item}.pkl"

        #Add link to queue scroller
        self.queue_scroller.add_item(item, "Run Link Independently")
        #Append the item namem to the names in queue list
        self.sheet_names_in_queue.append(str(item))
    #ADDS TRAINED LINK TO QUEUE SCROLLER

    def clearQueue(self):
        for theitem in self.sheet_names_in_queue:
            self.queue_scroller.remove_item(theitem)
    def runThisLink(self, item):
        print("RUNNING", item)

        filepath = fr'DATA_FROM_TRAINER/{str(item)}'
        #Open the pickle packet specified
        with open(filepath, 'rb') as f:
            unpickled_data = pickle.load(f)

            #Load data into attribute accesable from outside module
        self.queue_data = unpickled_data
        #return unpickled_data
    #Runs the fk data stored in pcikle packet named item

    def start_queue(self):

        requestDataFromQueue = []

        for sheet in self.sheet_names_in_queue:
            print(f"RUNNING {sheet} as apart of queue")

            filepath = fr'DATA_FROM_TRAINER/{str(sheet)}'
            #Open sheet and get data from sheet
            with open(filepath, 'rb') as f:
                unpickled_data = pickle.load(f)

            #Add the data from one link pickle packet into the queue list of data
            requestDataFromQueue.extend(unpickled_data)

        #Load data into attribute accesable from outside module
        self.queue_data = requestDataFromQueue

        #return requestDataFromQueue
    #Returns fk angles for all links in queue nested trice



class MyApp:

    def __init__(self, master, folder_path, init_angles, coord):
        self.master = master
        self.init_angles = init_angles

        self.trainer = TrainerWidget(self.master, folder_path, self.init_angles, coord)

    def run(self):
        self.master.mainloop()


if __name__ =="__main__":
    root = customtkinter.CTk()
    customtkinter.set_appearance_mode("dark")

    folder_path = r'DATA_FROM_TRAINER'
    coord = [20, 20]

    inst = MyApp(root,folder_path,[0, -17, 109, 0, 0, 0], coord)
    inst.run()

