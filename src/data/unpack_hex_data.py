import csv
import cv2 as cv

from cv_bridge.core import CvBridge

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, TopicMetadata

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from px4_msgs.msg import VehicleOdometry, GimbalDeviceAttitudeStatus
from hardware_msgs.msg import Flag # type: ignore
from sensor_msgs.msg import Image

odometry_fieldnames = ['bag_stamp', 'px4_stamp', 'px4_sample', 'pose_frame', 'velocity_frame',
                       'p_x', 'p_y', 'p_z', 'q_w', 'q_x', 'q_y', 'q_z',
                       'v_x', 'v_y', 'v_z', 'r_x', 'r_y', 'r_z']

gimbal_fieldnames = ['bag_stamp', 'px4_stamp', 'q_w', 'q_x', 'q_y', 'q_z']

video_fieldnames = ['bag_stamp', 'seconds', 'nanoseconds']

# path = '/DroneWorkspace/data/Prarie_Burn_2024/2024-09-18T00:08:09.917213_0'
# path = '/DroneWorkspace/data/Prarie_Burn_2024/2024-09-18T00:12:35.409295_0'
path = '/DroneWorkspace/data/Prarie_Burn_2024/2024-09-18T00:49:23.799975_0'

def main():
    reader = SequentialReader()
    bridge = CvBridge()

    storage_options = StorageOptions(
        uri = path,
        storage_id = 'sqlite3'
    )
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )

    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")
    
    
    video_writer = cv.VideoWriter(
        filename=path+'/video.mkv',
        apiPreference=cv.CAP_FFMPEG,
        fourcc=cv.VideoWriter_fourcc(*"FFV1"),
        fps=30,
        frameSize=(382, 288),
        params=[
            cv.VIDEOWRITER_PROP_DEPTH,
            cv.CV_16U,
            cv.VIDEOWRITER_PROP_IS_COLOR,
            0,  # false
        ]
    )
    
    odometry_csv = open(path+'/odometry.csv', 'w', newline='')
    odometry_writer = csv.DictWriter(odometry_csv, fieldnames=odometry_fieldnames)

    gimbal_csv = open(path+'/gimbal.csv', 'w', newline='')
    gimbal_writer = csv.DictWriter(gimbal_csv, fieldnames=gimbal_fieldnames)

    videoframe_csv = open(path+'/videoframes.csv', 'w', newline='')
    videoframe_writer = csv.DictWriter(videoframe_csv, fieldnames=video_fieldnames)

    odometry_writer.writeheader()
    gimbal_writer.writeheader()
    videoframe_writer.writeheader()

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(typename(topic))

        msg = deserialize_message(data, msg_type)

        if type(msg) == VehicleOdometry:
            x = msg.position
            q = msg.q
            v = msg.velocity
            r = msg.angular_velocity

            odometry_writer.writerow(
                {'bag_stamp': timestamp, 'px4_stamp': msg.timestamp, 'px4_sample': msg.timestamp_sample, 'pose_frame': msg.pose_frame, 'velocity_frame': msg.velocity_frame,
                 'p_x': x[0], 'p_y': x[1], 'p_z': x[2], 'q_w': q[0], 'q_x': q[1], 'q_y': q[2], 'q_z': q[3],
                 'v_x': v[0], 'v_y': v[1], 'v_z': v[2], 'r_x': r[0], 'r_y': r[1], 'r_z': r[2]}
            )
        
        if type(msg) == GimbalDeviceAttitudeStatus:
            q = msg.q
            
            gimbal_writer.writerow(
                {'bag_stamp': timestamp, 'px4_stamp': msg.timestamp, 'q_w': q[0], 'q_x': q[1], 'q_y': q[2], 'q_z': q[3]}
            )

        if type(msg) == Image:
            mat = bridge.imgmsg_to_cv2(msg)
            video_writer.write(mat)

            videoframe_writer.writerow({'bag_stamp': timestamp, 'seconds': msg.header.stamp.sec, 'nanoseconds': msg.header.stamp.nanosec})

    video_writer.release()
    odometry_csv.close()
    videoframe_csv.close()


if __name__ == "__main__":
    main()