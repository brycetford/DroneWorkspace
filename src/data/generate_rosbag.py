import os
import csv
import argparse
from pathlib import PurePosixPath

import rclpy
from rosbag2_py import SequentialWriter
from rosbag2_py._storage import StorageOptions, ConverterOptions, TopicMetadata


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
    
    return writer


def writeVehicleOdometry():
    pass


def writeVideo():
    pass


def writeVideoTransform():
    pass


def main(csvPath: PurePosixPath, mp4Path: PurePosixPath, lineupTime):

    # Get CSVs
    name = csvPath.stem
    dataPath = PurePosixPath(csvPath.parent)

    actuatorOutput = dataPath / (name + '_actuator_outputs_2.csv')
    homePosition = dataPath / (name + '_home_position_0.csv')
    estimatorSelectorStatus = dataPath / (name + '_estimator_selector_status.csv')

    estimatorStates_0 = dataPath / (name + '_estimator_states_0.csv')
    estimatorStates_1 = dataPath / (name + '_estimator_states_1.csv')
    estimatorStates_2 = dataPath / (name + '_estimator_states_2.csv')
    estimatorStates_3 = dataPath / (name + '_estimator_states_3.csv')

    # Create Bagwriter
    #bagwriter = createBag(name)


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