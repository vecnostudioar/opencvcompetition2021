import argparse
from typing import Type
import cv2
import bluetooth

import subprocess
import time
from pathlib import Path

import depthai as dai
import numpy as np
import time

import datetime
import os
from threading import Timer
import copy

print("Start script")

maxZDistance = 6000
np_arr1 = np.empty((0,4))
# [id], X, SumZ, CantZ
actualFps = 0
actualFpsVideo = 0
isRunning = True
currentTimeN = 0
currentTimeVF = 0
startTimeN = 0
startTimeVF = 0

# arguments
ap = argparse.ArgumentParser()
ap.add_argument("-r", "--record", help="If you want to save a OAK-D and reference video in videos folder", action='store_true')
ap.add_argument("-f", "--recordframes", help="If record Videos enable, set the frame rate ( default is 4 for better performance )", type=int, default=4)
ap.add_argument("-b", "--bluetoothname", help="The name of the bluetooth device to connect ( default is D-Gloves )", type=str, default="D-Gloves")
ap.add_argument("-i", "--invertbracelets", help="Invert Bracelets set signal left/right", action='store_true')
args = vars(ap.parse_args())
print (args)

# saveVideos = args["recordvideo"]
saveVideos = args['record']
saveVideosFrame = args['recordframes']

videoFrameCalc = round(1 / saveVideosFrame, 2)

invertbracelets = args['invertbracelets']

#bluetooth data
BtName = args['bluetoothname']      # Device name
port = 1         # RFCOMM port
passkey = "1234" # passkey of the device you want to connect

# kill any "bluetooth-agent" process that is already running
# subprocess.call("kill -9 `pidof bluetooth-agent`",shell=True)

# Start a new "bluetooth-agent" process where XXXX is the passkey
# status = subprocess.call("bluetooth-agent " + passkey + " &",shell=True)


driver_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

def connectBt():
    global port
    global isRunning
    global BtName
    if isRunning:
        print(f"trying to connect Bluetooth {BtName}...")
        
        # search bts
        nearby_devices = bluetooth.discover_devices(duration=1, lookup_names=True,
                                                    flush_cache=True, lookup_class=False)

        print("Found {} devices".format(len(nearby_devices)))

        conectionSuccess = False

        for addr, name in nearby_devices:
            try:
                print("   {} - {}".format(addr, name))
                if name == BtName:
                    # Now, connect in the same way as always with PyBlueZ
                    try:
                        
                        driver_socket.connect((addr,port))
                        conectionSuccess = True
                        print("Bluetooth connected!")

                    except bluetooth.btcommon.BluetoothError as err:
                        # Error handler
                        pass
                    

            except UnicodeEncodeError:
                print("   {} - {}".format(addr, name.encode("utf-8", "replace")))

        if not conectionSuccess:
            
            print("Not Bluetooth found.. trying again in 2 secs")
            t = Timer(2.0, connectBt)
            t.start() 


t = Timer(3.0, connectBt)
t.start() 


labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
            "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

# nnPathDefault = str((Path(__file__).parent / Path('models/mobilenet-ssd_openvino_2021.2_6shave.blob')).resolve().absolute())
nnPathDefault = 'mobilenet-ssd_openvino_2021.2_6shave.blob'
# nnPathDefault = 'mobilenet-ssd.blob'


# Perform tracking on full RGB frame
fullFrameTracking = False
nnPath = nnPathDefault

# Create pipeline
pipeline = dai.Pipeline()
pipeline.setOpenVINOVersion(dai.OpenVINO.Version.VERSION_2021_2)

# Define sources and outputs
camRgb = pipeline.createColorCamera()
spatialDetectionNetwork = pipeline.createMobileNetSpatialDetectionNetwork()
monoLeft = pipeline.createMonoCamera()
monoRight = pipeline.createMonoCamera()
stereo = pipeline.createStereoDepth()
objectTracker = pipeline.createObjectTracker()

xoutRgb = pipeline.createXLinkOut()
trackerOut = pipeline.createXLinkOut()

xoutRgb.setStreamName("preview")
trackerOut.setStreamName("tracklets")

# Properties
camRgb.setPreviewSize(300, 300)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

# setting node configs
stereo.setConfidenceThreshold(255)

spatialDetectionNetwork.setBlobPath(nnPath)
spatialDetectionNetwork.setConfidenceThreshold(0.5)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(5000)

# objectTracker.setDetectionLabelsToTrack([15])  # track only person
# objectTracker.setDetectionLabelsToTrack([12])  # track only dog
# objectTracker.setDetectionLabelsToTrack([7])  # track only car
# objectTracker.setDetectionLabelsToTrack([14])  # track only motorbike
# objectTracker.setDetectionLabelsToTrack([2])  # track only bicycle
objectTracker.setDetectionLabelsToTrack([7,14,2])
# possible tracking types: ZERO_TERM_COLOR_HISTOGRAM, ZERO_TERM_IMAGELESS
objectTracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
# take the smallest ID when new object is tracked, possible options: SMALLEST_ID, UNIQUE_ID
objectTracker.setTrackerIdAssigmentPolicy(dai.TrackerIdAssigmentPolicy.SMALLEST_ID)

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

camRgb.preview.link(spatialDetectionNetwork.input)
objectTracker.passthroughTrackerFrame.link(xoutRgb.input)
objectTracker.out.link(trackerOut.input)

if fullFrameTracking:
    camRgb.setPreviewKeepAspectRatio(False)
    camRgb.video.link(objectTracker.inputTrackerFrame)
    objectTracker.inputTrackerFrame.setBlocking(False)
    # do not block the pipeline if it's too slow on full frame
    objectTracker.inputTrackerFrame.setQueueSize(2)
else:
    spatialDetectionNetwork.passthrough.link(objectTracker.inputTrackerFrame)

spatialDetectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
spatialDetectionNetwork.out.link(objectTracker.inputDetections)
stereo.depth.link(spatialDetectionNetwork.inputDepth)




def calcBtData(x, z):
    global maxZDistance
    
    btIntesity = 0
    btDataToSend = "0,0;"

    if z < 3000:
        btIntesity = 3
    elif z < 4500:
        btIntesity = 2
    elif z < maxZDistance:
        btIntesity = 1

    centerLimit = 20

    if x > centerLimit:
        if invertbracelets:
            btDataToSend = f"0,{btIntesity};"
        else:
            btDataToSend = f"{btIntesity},0;"
    elif x < centerLimit * -1:
        if invertbracelets:
            btDataToSend = f"{btIntesity},0;"
        else:
            btDataToSend = f"0,{btIntesity};"
    else:
        btDataToSend = f"{btIntesity},{btIntesity};"

    return btDataToSend


def sendBtData(btData_):
    try:
        driver_socket.send(btData_)
    except:
        pass



# image
ImgHeight = 500
ImgWidth = 300

carImg = cv2.imread("carimage.png")
blank_image = cv2.imread("background_bicycle.png")



# video

now = datetime.datetime.now()
print(now.year, now.month, now.day, now.hour, now.minute, now.second)
videoName = f"{now.year}-{now.month}-{now.day}--{now.hour}{now.minute}{now.second}"

# fourcc = cv2.VideoWriter_fourcc(*'MPEG')
fourcc = cv2.VideoWriter_fourcc(*'avc1')

if saveVideos:
    videDir = os.path.join('videos', f"{videoName}.mp4")
    out = cv2.VideoWriter(videDir, fourcc, saveVideosFrame, (300,300))

    videDirImg = os.path.join('videos', f"{videoName}-reference.mp4")
    outImg = cv2.VideoWriter(videDirImg, fourcc, saveVideosFrame, (300,500))

newImage = copy.deepcopy(blank_image)

btDataInner = "0,0;"

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    preview = device.getOutputQueue("preview", 4, False)
    tracklets = device.getOutputQueue("tracklets", 4, False)

    startTime = time.monotonic()
    startTimeN = time.monotonic()
    startTimeVF = time.monotonic()
    counter = 0
    fps = 0
    color = (255, 0, 0)

    while(True):
        isHalfSecond = False
        imgFrame = preview.get()
        track = tracklets.get()
    
        currentTimeN = time.monotonic()
        currentTimeVF = time.monotonic()

        counter+=1
        current_time = time.monotonic()
        if (current_time - startTime) > 1 :
            fps = counter / (current_time - startTime)
            counter = 0
            startTime = current_time

        frame = imgFrame.getCvFrame()
        trackletsData = track.tracklets
        for t in trackletsData:
            roi = t.roi.denormalize(frame.shape[1], frame.shape[0])
            x1 = int(roi.topLeft().x)
            y1 = int(roi.topLeft().y)
            x2 = int(roi.bottomRight().x)
            y2 = int(roi.bottomRight().y)

            xr1 = int( x1 + (int(roi.bottomRight().x) - int(roi.topLeft().x)) / 2 )
            

            yr1 = int( y1 + (int(roi.bottomRight().y) - int(roi.topLeft().y)) / 2 )

            try:
                label = labelMap[t.label]
            except:
                label = t.label

            cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.3, 255)
            cv2.putText(frame, f"ID: {[t.id]}", (x1 + 10, y1 + 30), cv2.FONT_HERSHEY_TRIPLEX, 0.3, 255)
            cv2.putText(frame, t.status.name, (x1 + 10, y1 + 40), cv2.FONT_HERSHEY_TRIPLEX, 0.3, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)

            cv2.putText(frame, f"X: {int(t.spatialCoordinates.x)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.3, 255)
            cv2.putText(frame, f"Y: {int(t.spatialCoordinates.y)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.3, 255)
            cv2.putText(frame, f"Z: {int(t.spatialCoordinates.z)} mm", (x1 + 10, y1 + 95), cv2.FONT_HERSHEY_TRIPLEX, 0.3, 255)


            frame = cv2.line(frame, (xr1, yr1 - 10), (xr1, yr1+10), color, 2)
            frame = cv2.line(frame, (xr1 - 10, yr1), (xr1 + 10, yr1), color, 2)

            posZ = int(t.spatialCoordinates.z)
            

            if t.status.name == "TRACKED" and posZ != 0:

                hasInArray = False

                for x in np_arr1:
                    if x[0] == t.id:
                        #update exist
                        hasInArray = True
                        x[2] += posZ
                        x[3] = x[3] + 1
                        x[1] = xr1


                if not hasInArray:
                    np_arr1 = np.append(np_arr1, np.array([[t.id, xr1, posZ, 1]]), axis=0)


        if currentTimeN - startTimeN > 0.5:
            startTimeN = currentTimeN
            isHalfSecond = True


        cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, 10), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)


        if isHalfSecond:

            newImage = copy.copy(blank_image)
            ln = len(np_arr1)
            if ln > 0:

                #draw detection image referente
                for xa in np_arr1:
                    zCalc = int( xa[2] / xa[3] )

                    if zCalc < maxZDistance:

                        x_offset = 300 - (int(xa[1]) + 20)   #0 -> 300
                        y_offset = int( zCalc * 410 / maxZDistance ) + 90 #0 -> 500
                        if x_offset <= 300 and y_offset <= 500:
                            try:
                                newImage[y_offset:y_offset+carImg.shape[0], x_offset:x_offset+carImg.shape[1]] = carImg
                            except:
                                pass


                zCalc = int( np_arr1[0][2] / np_arr1[0][3] )
                xr = ( np_arr1[0][1] ) - 150

                btDataInner = calcBtData(xr, zCalc)

                sendBtData(btDataInner)
                np_arr1 = np.delete(np_arr1, 0, 0)
            else:
                btDataInner = "0,0;"

            cv2.putText(frame, btDataInner, (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.6, 255)
            
            sendBtData(btDataInner)



        cv2.putText(frame, btDataInner, (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.6, 255)


        liniecolor=(0, 255, 0)
        start_point=(130, 0)
        end_point=(130, 300)
        frame = cv2.line(frame, start_point, end_point, liniecolor, thickness=1)
        start_point=(170, 0)
        end_point=(170, 300)
        frame = cv2.line(frame, start_point, end_point, liniecolor, thickness=1)


        difTime = round(currentTimeVF - startTimeVF, 2)
        if difTime >= videoFrameCalc:
            startTimeVF = currentTimeVF
            if saveVideos:
                out.write(frame)
                outImg.write(newImage)


        cv2.imshow("AI cyclist Guardian", frame)
        cv2.imshow("AI cyclist Guardian Reference", newImage)

        if cv2.waitKey(1) == ord('q'):
            try:
                driver_socket.send("6,6;")
                driver_socket.close()
                print("Bluetooth desconnected..")
            except:
                print("cant send desconection signal")
            
            if saveVideos:
                out.release()
                # out10.release()
                outImg.release()
            isRunning = False

            break


