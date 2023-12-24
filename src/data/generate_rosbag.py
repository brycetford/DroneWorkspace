import os
import csv
import argparse
from pathlib import PurePosixPath
from tqdm import tqdm

import numpy as np

import cv2

import rclpy
from rclpy.serialization import serialize_message
from rclpy.time import Time
from rosbag2_py import SequentialWriter
from rosbag2_py._storage import StorageOptions, ConverterOptions, TopicMetadata

from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleOdometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion

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
        name='/fmu/out/estimator_states',
        type='px4_msgs/msg/VehicleOdometry',
        serialization_format='cdr')
    
    attitudeTopic = TopicMetadata(
        name='/fmu/out/vehicle_attitude',
        type='px4_msgs/msg/VehicleAttitude',
        serialization_format='cdr')
    
    positionTopic = TopicMetadata(
        name='/fmu/out/vehicle_local_position',
        type='px4_msgs/msg/VehicleLocalPosition',
        serialization_format='cdr')

    videoTopic = TopicMetadata(
        name='/camera/ir/image',
        type='sensor_msgs/msg/Image',
        serialization_format='cdr')
    
    videoTransformTopic = TopicMetadata(
        name='/camera/transform',
        type='geometry_msgs/msg/Quaternion',
        serialization_format='cdr')
    
    writer.create_topic(odometryTopic)
    writer.create_topic(attitudeTopic)
    writer.create_topic(positionTopic)
    writer.create_topic(videoTopic)
    writer.create_topic(videoTransformTopic)
    
    return writer


def writeVehicleOdometry(bagWriter, estimatorList, currentEstimator):
    
    print("Reading Vehicle Odometry")

    odometryMsgList = []

    for estimator in currentEstimator:

        startTime = estimator[0]
        endTime = estimator[1]
        estimatorIndex = estimator[2]

        # Open the current estimator log 
        with open(file=estimatorList[estimatorIndex], mode='r', newline='') as csvfile:

            reader = csv.DictReader(csvfile)

            # Read all rows and create Odometry msgs for rows in valid time range for active estimator
            for row in reader:

                if int(row['timestamp']) < startTime:
                    continue

                if int(row['timestamp']) > endTime:
                    break

                odometryMsgList.append(createOdometryMsg(row))


    print("Writing Odometry to Bag:")
    
    for msg in tqdm(odometryMsgList):

        # Convert timestamp in microseconds to nanoseconds
        nanos = int(msg.timestamp * 1e3)

        bagWriter.write(
            '/fmu/out/estimator_states',
            serialize_message(msg),
            nanos
        )


def writeVehicleAttitude(bagWriter, vehicleAttitude):

    print("Reading Vehicle Attitude")

    attitudeMessageList = []

    with open(file=vehicleAttitude, mode='r', newline='') as csvfile:

        reader = csv.DictReader(csvfile)

        # Read all rows and create Attitude msgs for rows
        for row in reader: attitudeMessageList.append(createAttitudeMsg(row))

    print("Writing Attitude to Bag:")
    
    for msg in tqdm(attitudeMessageList):

        # Convert timestamp in microseconds to nanoseconds
        nanos = int(msg.timestamp * 1e3)

        bagWriter.write(
            '/fmu/out/vehicle_attitude',
            serialize_message(msg),
            nanos
        )


def writeVehiclePosition(bagWriter, vehicleLocalPosition):
    
    print("Reading Vehicle Position")

    localPositionMessageList = []

    with open(file=vehicleLocalPosition, mode='r', newline='') as csvfile:

        reader = csv.DictReader(csvfile)

        for row in reader: localPositionMessageList.append(createLocalPositionMsg(row))

    print("Writing Local Position to Bag:")

    for msg in tqdm(localPositionMessageList):

        # Convert timestamp in microseconds to nanoseconds
        nanos = int(msg.timestamp * 1e3)

        bagWriter.write(
            '/fmu/out/vehicle_local_position',
            serialize_message(msg),
            nanos
        )


def writeVideoTransform():
    pass


def writeVideo():
    pass


# Creates a vehicle odometry message using an ekf states log row
def createOdometryMsg(row: dict) -> VehicleOdometry:
    msg = VehicleOdometry()

    # Timestamp
    msg.timestamp = int(row['timestamp'])
    msg.timestamp_sample = int(row['timestamp_sample'])

    # Position
    msg.position[0] = float(row['states[7]'])
    msg.position[1] = float(row['states[8]'])
    msg.position[2] = float(row['states[9]'])

    # Velocity
    msg.velocity[0] = float(row['states[4]'])
    msg.velocity[1] = float(row['states[5]'])
    msg.velocity[2] = float(row['states[6]'])

    # Orientation
    msg.q[0] = float(row['states[0]'])
    msg.q[1] = float(row['states[1]'])
    msg.q[2] = float(row['states[2]'])
    msg.q[3] = float(row['states[3]'])

    # Position Variance
    msg.position_variance[0] = float(row['covariances[6]'])
    msg.position_variance[1] = float(row['covariances[7]'])
    msg.position_variance[2] = float(row['covariances[8]'])

    # Velocity Variance
    msg.velocity_variance[0] = float(row['covariances[3]'])
    msg.velocity_variance[1] = float(row['covariances[4]'])
    msg.velocity_variance[2] = float(row['covariances[5]'])

    # Orientation Variance
    msg.orientation_variance[0] = float(row['covariances[0]'])
    msg.orientation_variance[1] = float(row['covariances[1]'])
    msg.orientation_variance[2] = float(row['covariances[2]'])

    return msg


def createAttitudeMsg(row: dict) -> VehicleAttitude:

    msg = VehicleAttitude()

    msg.timestamp = int(row['timestamp'])
    msg.timestamp_sample = int(row['timestamp_sample'])

    # Orientation
    msg.q[0] = float(row['q[0]'])
    msg.q[1] = float(row['q[1]'])
    msg.q[2] = float(row['q[2]'])
    msg.q[3] = float(row['q[3]'])

    # Orientation delta of last reset
    msg.delta_q_reset[0] = float(row['delta_q_reset[0]'])
    msg.delta_q_reset[1] = float(row['delta_q_reset[1]'])
    msg.delta_q_reset[2] = float(row['delta_q_reset[2]'])
    msg.delta_q_reset[3] = float(row['delta_q_reset[3]'])

    # Num Resets
    msg.quat_reset_counter = int(row['quat_reset_counter'])

    return msg

def createLocalPositionMsg(row: dict) -> VehicleLocalPosition:

    msg = VehicleLocalPosition()

    msg.timestamp = int(row['timestamp'])
    msg.timestamp_sample = int(row['timestamp_sample'])

    # Position
    msg.x = float(row['x'])
    msg.y = float(row['y'])
    msg.z = float(row['z'])

    # Position delta if last reset
    msg.delta_xy[0] = float(row['delta_xy[0]'])
    msg.delta_xy[1] = float(row['delta_xy[1]'])
    msg.delta_z = float(row['delta_z'])

    msg.xy_reset_counter = int(row['xy_reset_counter'])
    msg.z_reset_counter = int(row['z_reset_counter'])

    # Global Reference
    msg.ref_lat = float(row['ref_lat'])
    msg.ref_lon = float(row['ref_lon'])
    msg.ref_alt = float(row['ref_alt'])

    msg.ref_timestamp = int(row['ref_timestamp'])
    msg.xy_global = bool(row['xy_global'])
    msg.z_global = bool(row['z_global'])


    return msg


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

    # Get vehicle attitude and position logs
    vehicleAttitude = dataPath / (name + '_vehicle_attitude_0.csv')
    vehicleLocalPosition = dataPath / (name + '_vehicle_local_position_0.csv')

    # Get current estimator list
    with open(file=estimatorSelectorStatus, mode='r', newline='') as csvfile:

        reader = csv.DictReader(csvfile)

        estimator = 0
        timestamp = 0

        newEstimator = 0
        newTimestamp = 0

        currentEstimator = []

        for row in reader:
            newEstimator = int(row['primary_instance'])
            newTimestamp = int(row['timestamp'])
            if estimator != newEstimator:
                currentEstimator.append(
                    (timestamp, newTimestamp, estimator)
                )
                estimator = newEstimator
                timestamp = newTimestamp

        currentEstimator.append(
            (timestamp, newTimestamp, newEstimator)
        )
            
    # Create Bagwriter
    bagWriter = createBag(name)

    # Write Vehicle Odometry
    writeVehicleOdometry(bagWriter, estimatorList, currentEstimator)

    # Write Vehicle Attitude
    writeVehicleAttitude(bagWriter, vehicleAttitude)

    # Write Vehicle Local Position
    writeVehiclePosition(bagWriter, vehicleLocalPosition)

    # Write Gimbal Transform


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