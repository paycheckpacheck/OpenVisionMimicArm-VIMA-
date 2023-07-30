import serial.serialutil
from serial import Serial
import keyboard
import numpy as np
import time


'''

Pacheck, Nicholas
OpenVIMA: OpenVIsionMimicArm, SERIAL_SENDER

SERIAL_SENDER is a module that takes the calculated angles from the 
FORWARD_KINEMATICS_MODULE, INVERSE_KINEMATICS_MODULE, AND THE COMPUTER_VISION_MODULE
and sends them over a serial connection.

'''


class Arduino:

    '''
    The arduino will be configured such that it is ready to send
    commands back to this script with the string READY.

    Arudino will go to angle requested. meaning

    '''
    def __init__(self, port, baudrate):
        self.baud  = int(baudrate)
        self.port = str(port)
        self.ser = Serial(self.port, self.baud, timeout = 0.1)
        self.start_timer = time.time()

        #List that stores the buffer for if a process
        self.bufferList = []

        self.ready = False
        self.my_zfill_num = 4



        self.connect_to_arduino(self.port, self.baud)

    def send_data(self, data, reset):
        """Send data to MainArduino:
        data = [Base, Shoulder, Elbow, GAMMA, ALPHA, PHI, EMERGENCY_BOOL]"""
        try:
            data =  bytes(data, 'utf-8')

            if reset:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                #IN TEST
                #self.ser.cancel_write()

            #from_arduino = self.get_data()



            self.ser.write(data)
                #print(self.ser, "ser")
        except TypeError as TE:
            print(TE)


    def inLoopSender2(self, requestList):


        '''This function will be the all in one handler for Serial communication:

        It will format the dataWrist and ROborData into one long string, then send the data to arduino.
         This will be called frm within the main loop where new dataWrist ad robot data are updating within loop


         Finish this function such that the next finalstring message doesntsend until the arduino sends back
         a string'''

        '''
        # Format data and append to buffer list
        finalString =  ''.join(requestList)

        self.bufferList.append(finalString)



        # Read Arduino data
        response = str(self.get_data())

        # If response is empty, return
        if response is None:
            print(response, "ISSUE: ARDUINO RESPONSE MEPTY")

            return  "0000000000000000000000001"


        # Check if Arduino is ready for new data
        if "READY" in response:
            # Remove "READY" from response
            response = response.replace("READY", "").strip()

            # If buffer list is empty, return
            if not self.bufferList:
                return self.ready

            # Send next message in buffer list
            self.send_data(self.bufferList[0])

            # Remove sent message from buffer list
            self.bufferList.pop(0)

            # Set ready state to False
            self.ready = False

        return str(response)'''



    def bufferReset(self):

        for i in range(3):
            time.sleep(1)
            print(f"RESETTING BUFFER,    "
                  f"RESETTING BUFFER: {self.bufferList}"
                  f"RESETTNG BUFFER   ")


        #Send flush command to arduino
        while not "READY" in str(self.get_data()):
            self.send_data("F")
            time.sleep(1)
            print("FLushhing")


        buffer_timer = time.time()
        time2Buffer = abs(buffer_timer - self.start_timer)

        # print(time2Buffer)
        #RESET serial buffer
        self.ser.reset_input_buffer()  # clears the input buffer\

        #RESET Software buffer
        self.bufferList = []

        for i in range(3):
            time.sleep(1)
            print("RESETTING BUFFER,    RESETTNG BUFFER,    RESETTING BUFFER", time2Buffer)





    def inLoopSender(self, requestList, reset):
        '''
            This function should be called as the main serial sending
            function as it handles all serial stuff through the ussage of self.class//self.__name__

            input the requestList length 6 for 6 dof and check for sending emergecies from hgh level
        passes the reset variable to the send funciton descirbed above


        TODO: FEEDBACK FROM ARDUINO

        '''

        #FILL ANGLES WITH ZEROS TO MATCH THE CUSTOM ARUDINO SERIAL PROTOCAL
        formatted_numbers = [str(num).zfill(self.my_zfill_num) for num in requestList]

        #COMBINE INTO STRING
        finalString = ''.join((formatted_numbers))
        #PARSE FOR CUSTOM SERIAL COMMUNICATIONS, CUSTOM START AND STOP BYTES
        finalString = 'A' + finalString + 'B'
        print(f"               Sending {finalString}")

        #SEND DATA
        self.send_data(finalString, reset)


    def flush(self):
        self.ser.flush()
        print("flushed")

    def get_data(self):

        my_string  = self.ser.readline()
        #print(my_string, "i got", (my_string.decode('utf-8')))
        return my_string


    def connect_to_arduino(self, port, baudrate):
        try:
            #maybe implement match case exception for finding arduino serial


            while not self.ser.is_open:  #retry to init serial

                self.ser.open()

        except serial.serialutil.SerialException as SerialError:
            print("Serial Error:", SerialError)
            #retry to init serial
            self.__init__(self.port, self.baud)
            pass



class Switch:
    def __init__(self):
        self.value = False
        keyboard.on_press_key('a', self.toggle)

    def toggle(self, event):
        if event.name == 'a':
            self.value = not self.value
            print('Switch is now', self.value)



'''

arduino = Arduino("COM6", 9600)

string = "0900900900900900901\n"
string = string#.encode('ascii')

string2 = "0901800900900900901\n"



while True:
    arduino.send_data(bytes(string, 'utf-8'))

    time.sleep(1)

    arduino.get_data()

    arduino.send_data(bytes(string2, 'utf-8'))

'''