import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from random import randint

import COMPUTER_VISION_MODULE as pacheckCV
import time
import cv2

from matplotlib.figure import Figure

# make numpy raise errors instead of warn so can be caught by try except blocks
np.seterr(all="raise")
hover = "#CF4420"

plt.ion()

class ForwardKinematics:

    def __init__(self, limits):
        #self.vision = pacheckCV.ComputerVisionBody(False)


        # Create a new figure and add a 3D subplot
        self.fig = plt.figure(figsize=(10,8)) #stable

        plt.style.use('dark_background')

        plt.subplots_adjust(left=0,
                            bottom=-0.7,
                            right=0.917,
                            top=1,
                            wspace=0.706,
                            hspace=0.5)
        #DEV
        #self.fig = Figure(figsize=(20,16))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.limits = limits

        pass

    # plot axis origins onto point with rotation
    def plotOrigin(self, tx, ty, tz, rx, ry, rz, length):
        # transform origin
        rotall = np.matmul(np.matmul(self.Rz(rz), self.Ry(ry)), self.Rx(rx))
        xplots = np.matmul(rotall, ((0, length), (0, 0), (0, 0)))
        yplots = np.matmul(rotall, ((0, 0), (0, length), (0, 0)))
        zplots = np.matmul(rotall, ((0, 0), (0, 0), (0, length)))
        xplots[0] += tx
        xplots[1] += ty
        xplots[2] += tz
        yplots[0] += tx
        yplots[1] += ty
        yplots[2] += tz
        zplots[0] += tx
        zplots[1] += ty
        zplots[2] += tz

        # plot origin
        self.ax.plot(*xplots, c="#ff0000")
        self.ax.plot(*yplots, c="#00ff00")
        self.ax.plot(*zplots, c="#0000ff")

    # find the position of point given a vector
    def findPoint(self, start, length, theta1, theta2):
        end = [0, 0, 0]
        end[0] = start[0] + length * np.cos(theta1) * np.cos(theta2)
        end[1] = start[1] + length * np.cos(theta1) * np.sin(theta2)
        end[2] = start[2] + length * np.sin(theta1)
        return end

    # define rotation matrices
    def Rx(self, a):
        return ((1, 0, 0), (0, np.cos(a), -np.sin(a)), (0, np.sin(a), np.cos(a)))

    def Ry(self, a):
        return ((np.cos(a), 0, np.sin(a)), (0, 1, 0), (-np.sin(a), 0, np.cos(a)))

    def Rz(self, a):
        return ((np.cos(a), -np.sin(a), 0), (np.sin(a), np.cos(a), 0), (0, 0, 1))

    # function to find angle between two vectors
    def getAngle(self, vect1, vect2):
        try:
            temp1 = np.linalg.norm(vect1)
            temp2 = np.linalg.norm(vect2)
            unitvect1 = vect1 / temp1
            unitvect2 = vect2 / temp2
            theta = np.arccos(np.clip(np.dot(unitvect1, unitvect2), -1.0, 1.0))
            return theta
        except Exception:
            return np.pi / 2 if temp2 > 0 else -np.pi / 2

    # linear interpolation
    def lerp(self, a, b, t):
        return (a * (1 - t)) + (b * t)

    def fwdKin(self, lengths, angles):

        try:
            # trace the path to find plot points
            xplots = [0, 0]
            yplots = [0, 0]
            zplots = [0, lengths[0]]
            mainlength1 = lengths[1]
            mainlength2 = lengths[2] + lengths[3]
            mainlength3 = lengths[4] + lengths[5]
            theta1 = (np.pi / 2) - angles[1]
            theta2 = angles[0]
            point = self.findPoint([xplots[1], yplots[1], zplots[1]], mainlength1, theta1, theta2)
            xplots.append(point[0])
            yplots.append(point[1])
            zplots.append(point[2])
            theta1 = (np.pi / 2) - angles[1] - angles[2]
            theat2 = angles[0]
            point = self.findPoint([xplots[2], yplots[2], zplots[2]], mainlength2, theta1, theta2)
            savedpoint = point  # save the point for later derivation
            xplots.append(self.lerp(xplots[2], point[0], lengths[2] / mainlength2))
            yplots.append(self.lerp(yplots[2], point[1], lengths[2] / mainlength2))
            zplots.append(self.lerp(zplots[2], point[2], lengths[2] / mainlength2))
            xplots.append(point[0])
            yplots.append(point[1])
            zplots.append(point[2])
            transformed = np.array([mainlength3, 0, 0])
            transformed = np.matmul(self.Ry(angles[4]), transformed)
            transformed = np.matmul(self.Rx(angles[3]), transformed)
            transformed = np.matmul(self.Ry(np.pi + theta1), transformed)
            transformed = np.matmul(self.Rz(np.pi + theta2), transformed)
            point = transformed + point
            xplots.append(self.lerp(xplots[4], point[0], lengths[4] / mainlength3))
            yplots.append(self.lerp(yplots[4], point[1], lengths[4] / mainlength3))
            zplots.append(self.lerp(zplots[4], point[2], lengths[4] / mainlength3))
            xplots.append(point[0])
            yplots.append(point[1])
            zplots.append(point[2])

            # convert the data type
            xplots = np.array(xplots)
            yplots = np.array(yplots)
            zplots = np.array(zplots)

            # we now know the end position
            tx = xplots[-1]
            ty = yplots[-1]
            tz = zplots[-1]

            # derive theta1 and theta2 of wrist vector
            # define root vectors
            wrstvect = point - savedpoint

            txdelta = wrstvect[0]
            tydelta = wrstvect[1]
            tzdelta = wrstvect[2]

            elevvect = np.array([txdelta, tydelta, 0])
            azimvect = np.array([txdelta, 0, tzdelta])

            elevangle = self.getAngle(elevvect, wrstvect)
            azimangle = self.getAngle(azimvect, wrstvect)

            if tzdelta < 0:
                elevangle *= -1

            if tydelta < 0:
                azimangle *= -1

            rx = angles[5]
            ry = -elevangle
            rz = azimangle

            return ((tx, ty, tz, rx, ry, rz), xplots, yplots, zplots)
        except IndexError as IE:
            print(IE)
            pass

    def main(self, angles):
        angles = np.array(angles)

        self.ax.clear()

        # CONFIGS
        self.ax.w_xaxis.line.set_color(hover)
        self.ax.w_yaxis.line.set_color(hover)
        self.ax.w_zaxis.line.set_color(hover)

        self.ax.set_xlabel("x-axis")
        self.ax.set_ylabel("y-axis")
        self.ax.set_zlabel("z-axis")

        # SET AXIXS LIMITS SUCH THAT BASIS DOESNT CHANGE FROM FRAME2FRAME
        # self.limits = [-200, 200]
        self.ax.set_xlim(self.limits)
        self.ax.set_ylim(self.limits)
        self.ax.set_zlim(self.limits)

        # self.ax.view_init(elev=30, azim=-135)
        # self.ax.view_init(elev=30, azim=135)

        lengths = np.array([50.0, 200.0, 100.0, 100.0, 25.0, 25.0])

        raw = self.fwdKin(lengths, np.radians(angles))
        tx = raw[0][0]
        ty = raw[0][1]
        tz = raw[0][2]
        rx = raw[0][3]
        ry = raw[0][4]
        rz = raw[0][5]
        xplots = raw[1]
        yplots = raw[2]
        zplots = raw[3]

        # plot data
        self.ax.plot(xplots, yplots, zplots, c= "#630031")


        # update line width of all lines in the figure
        for line in self.ax.get_lines():
            line.set_linewidth(12)

        self.ax.scatter(xplots, yplots, zplots, c=hover)
        self.plotOrigin(xplots[0], yplots[0], zplots[0], 0, 0, 0, 50)
        self.plotOrigin(xplots[6], yplots[6], zplots[6], rx, ry, rz, 50)

        # show plotted data
        plt.draw()
        plt.pause(0.4)


    def main1(self, angles):

        angles = np.array(angles)


        self.ax.clear()

        #CONFIGS
        self.ax.w_xaxis.line.set_color(hover)
        self.ax.w_yaxis.line.set_color("#00ff00")
        self.ax.w_zaxis.line.set_color("#0000ff")



        self.ax.set_xlabel("x-axis")
        self.ax.set_ylabel("y-axis")
        self.ax.set_zlabel("z-axis")

        #SET AXIXS LIMITS SUCH THAT BASIS DOESNT CHANGE FROM FRAME2FRAME
        #self.limits = [-200, 200]
        self.ax.set_xlim(self.limits)
        self.ax.set_ylim(self.limits)
        self.ax.set_zlim(self.limits)

        # self.ax.view_init(elev=30, azim=-135)
        #self.ax.view_init(elev=30, azim=135)

        lengths = np.array([50.0, 200.0, 100.0, 100.0, 25.0, 25.0])

        raw = self.fwdKin(lengths, np.radians(angles))
        tx = raw[0][0]
        ty = raw[0][1]
        tz = raw[0][2]
        rx = raw[0][3]
        ry = raw[0][4]
        rz = raw[0][5]
        xplots = raw[1]
        yplots = raw[2]
        zplots = raw[3]

        # plot data
        self.ax.plot(xplots, yplots, zplots, c=hover)
        self.ax.scatter(xplots, yplots, zplots, c="#00ffff")
        self.plotOrigin(xplots[0], yplots[0], zplots[0], 0, 0, 0, 50)
        self.plotOrigin(xplots[6], yplots[6], zplots[6], rx, ry, rz, 50)
        #self.fig.canvas.draw()



        # show plotted data
        plt.draw()
        plt.pause(0.5)

    def main2(self, angles):
        angles = np.array(angles)

        # get current axis object and configure plot
        self.ax = plt.gca(projection="3d")
        self.ax.set_xlabel("x-axis")
        self.ax.set_ylabel("y-axis")
        self.ax.set_zlabel("z-axis")
        self.ax.view_init(elev=30, azim=135)
        self.ax.w_xaxis.line.set_color("#ff0000")
        self.ax.w_yaxis.line.set_color("#00ff00")
        self.ax.w_zaxis.line.set_color("#0000ff")

        lengths = np.array([50.0, 200.0, 100.0, 100.0, 25.0, 25.0])

        raw = self.fwdKin(lengths, np.radians(angles))
        tx = raw[0][0]
        ty = raw[0][1]
        tz = raw[0][2]
        rx = raw[0][3]
        ry = raw[0][4]
        rz = raw[0][5]
        xplots = raw[1]
        yplots = raw[2]
        zplots = raw[3]


        # plot data with updated styles
        self.ax.plot(xplots, yplots, zplots, c="#000000", linestyle='--')

        for line in self.ax.get_lines():
            line.set_linewidth(30)


        self.ax.scatter(xplots, yplots, zplots, c="#00ffff", marker='o')
        self.plotOrigin(xplots[0], yplots[0], zplots[0], 0, 0, 0, 50)
        self.plotOrigin(xplots[6], yplots[6], zplots[6], rx, ry, rz, 50)



        # show updated plot without blocking
        plt.draw()
        #plt.show(block=False)


if __name__ == "__main__":
    FK = ForwardKinematics()

    while True:
        #newThetaList, personBool, img, shoulderBool = pacheckCV.inLoop_vision_body(FK.vision)

        #print(len(newThetaList)

        newThetaList = [randint(0, 180),
                        randint(0, 180),
                        randint(0, 180),
                        randint(0, 180),
                        randint(0, 180),
                        randint(0, 180)]

        FK.main(newThetaList)

        #if personBool and shoulderBool and len(newThetaList)==6:
            #FK.ax.clear()
            #FK.main(newThetaList)
            #plt.show(FK.fig)

        #cv2.imshow("img",img)
        #cv2.waitKey(1)

        #time.sleep(0.5)