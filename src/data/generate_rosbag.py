import os
import csv
import argparse
from pathlib import PurePosixPath

import rclpy
from rosbag2_py import SequentialWriter
from rosbag2_py._storage import StorageOptions, ConverterOptions, TopicMetadata

from px4_msgs.msg import VehicleOdometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped

# This script takes a set of csv names, a mp4 name, and an lineup time an creates a rosbag

# Creates new Rosbag and opens it
def createBag(name):

    # Create new bag object
    writer = SequentialWriter()

    # Create bag metadata
    storageOptions = StorageOptions(uri=name, storage_id='sqlite3')    
    converterOptions = ConverterOptions('', '')

    # Open the bag
    writer.open(storageOptions, converterOptions)

    # Create the topics
    odometryTopic = TopicMetadata(
        name='/fmu/out/vehicle_odometry',
        type='px4_msgs/msg/VehicleOdometry',
        serialization_format='cdr')
    
    videoTopic = TopicMetadata(
        name='/camera/ir/image',
        type='sensor_msgs/msg/Image',
        serialization_format='cdr')
    
    videoTransformTopic = TopicMetadata(
        name='/camera/transform',
        type='geometry_msgs/msg/TransformStamped',
        serialization_format='cdr')
    
    writer.create_topic(odometryTopic)
    writer.create_topic(videoTopic)
    writer.create_topic(videoTransformTopic)
    
    return writer


def writeVehicleOdometry(bagWriter, estimatorList, currentEstimator):
    
    print("Reading Vehicle Odometry")

    odometryMsgList = []

    for estimator in currentEstimator:

        # Open the current estimator log 
        with open(file=estimatorList[estimator[1]], mode='r', newline='') as csvfile:

            reader = csv.DictReader(csvfile)

            readerTimestamp = -1


def writeVideo():
    pass


def writeVideoTransform():
    pass


def main(csvPath: PurePosixPath, mp4Path: PurePosixPath, lineupTime: float):

    # Get CSVs
    name = csvPath.stem
    dataPath = PurePosixPath(csvPath.parent)

    actuatorOutput = dataPath / (name + '_actuator_outputs_2.csv')
    homePosition = dataPath / (name + '_home_position_0.csv')
    estimatorSelectorStatus = dataPath / (name + '_estimator_selector_status_0.csv')

    # Hardcoded 4 estimators for now, I think this will be fine for all of my datasets
    estimatorStates_0 = dataPath / (name + '_estimator_states_0.csv')
    estimatorStates_1 = dataPath / (name + '_estimator_states_1.csv')
    estimatorStates_2 = dataPath / (name + '_estimator_states_2.csv')
    estimatorStates_3 = dataPath / (name + '_estimator_states_3.csv')

    estimatorList = [estimatorStates_0, estimatorStates_1, estimatorStates_2, estimatorStates_3]

    # Get current estimator list
    currentEstimator = []
    with open(file=estimatorSelectorStatus, mode='r', newline='') as csvfile:

        reader = csv.DictReader(csvfile)

        estimator = -1

        for row in reader:
            newEstimator = int(row['primary_instance'])
            if estimator != newEstimator:
                currentEstimator.append(
                    (int(row['timestamp']), newEstimator)
                )
                estimator = newEstimator
            
    # Create Bagwriter
    bagWriter = createBag(name)

    # Write Vehicle Odometry
    writeVehicleOdometry(bagWriter, estimatorList, currentEstimator)


if __name__ == "__main__":

    # Create and parse program args
    parser = argparse.ArgumentParser(prog='GenRosbag')

    parser.add_argument('csvPath', type=str)
    parser.add_argument('mp4Path', type=str)
    parser.add_argument('lineupTime', type=float, default=0.0)

    args = parser.parse_args()

    csvPath = PurePosixPath(args.csvPath)
    mp4Path = PurePosixPath(args.mp4Path)
    lineupTime = args.lineupTime

    # Enter Main
    main(csvPath, mp4Path, lineupTime)