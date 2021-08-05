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

maxZDistance = 6000
np_arr1 = np.empty((0,4))
# [id], X, SumZ, CantZ
actualFps = 0
actualFpsVideo = 0
isRunning = True
currentTimeN = 0
startTimeN = 0

# arguments
ap = argparse.ArgumentParser()
ap.add_argument("-r", "--record", help="If you want to save a OAK-D and reference video", action='store_true')
args = vars(ap.parse_args())
print (args)

# saveVideos = args["recordvideo"]
saveVideos = False

print(f"saveVideo: {saveVideos}")

#bluetooth data
BtName = "D-Gloves"      # Device name
port = 1         # RFCOMM port
passkey = "1234" # passkey of the device you want to connect

driver_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

def connectBt():
    global port
    global isRunning
    global BtName
    if isRunning:
        print("trying to connect Bluetooth...")
        
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

parser = argparse.ArgumentParser()
parser.add_argument('nnPath', nargs='?', help="Path to mobilenet detection network blob", default=nnPathDefault)
parser.add_argument('-ff', '--full_frame', action="store_true", help="Perform tracking on full RGB frame", default=False)

args = parser.parse_args()

fullFrameTracking = args.full_frame

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

spatialDetectionNetwork.setBlobPath(args.nnPath)
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






