#!/usr/bin/env python3

import rosbag2_py

from rclpy.serialization import deserialize_message

from crab_interfaces.msg import LoadCell
from crab_interfaces.msg import ServoData

import matplotlib.pyplot as plt
import numpy as np


# =====================================================
# CONFIG
# =====================================================

BAG_PATH = "/home/odinroast/crab_ws/adya_gait3_both.bag"

LOAD_TOPIC = "/load_cell_data"
CMD_TOPIC = "/servo/position_data"
ENC_TOPIC = "/servo/encoder_data"


# =====================================================
# Helpers
# =====================================================

def stamp_to_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def normalize_time(t):
    if len(t) == 0:
        return t

    return t - t[0]


# =====================================================
# Read bag
# =====================================================

def read_bag():

    storage_options = rosbag2_py.StorageOptions(
        uri=BAG_PATH,
        storage_id="mcap"
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )


    reader = rosbag2_py.SequentialReader()

    reader.open(
        storage_options,
        converter_options
    )


    load_times = []
    load_data = []

    cmd_times = []
    cmd_data = []

    enc_times = []
    enc_data = []


    while reader.has_next():

        topic, data, _ = reader.read_next()


        if topic == LOAD_TOPIC:

            msg = deserialize_message(
                data,
                LoadCell
            )

            load_times.append(
                stamp_to_seconds(msg.header.stamp)
            )


            matrix = np.array(
                msg.data,
                dtype=np.float32
            )


            matrix = matrix.reshape(
                msg.rows,
                msg.cols
            )


            # matrix is 20x6
            load_data.append(matrix)



        elif topic == CMD_TOPIC:

            msg = deserialize_message(
                data,
                ServoData
            )


            cmd_times.append(
                stamp_to_seconds(msg.header.stamp)
            )

            cmd_data.append(
                msg.data
            )



        elif topic == ENC_TOPIC:

            msg = deserialize_message(
                data,
                ServoData
            )


            enc_times.append(
                stamp_to_seconds(msg.header.stamp)
            )

            enc_data.append(
                msg.data
            )



    return (
        np.array(load_times),
        np.array(load_data),

        np.array(cmd_times),
        np.array(cmd_data),

        np.array(enc_times),
        np.array(enc_data)
    )



# =====================================================
# Main
# =====================================================

def main():

    (
        load_t,
        load_data,

        cmd_t,
        cmd_data,

        enc_t,
        enc_data

    ) = read_bag()



    load_t = normalize_time(load_t)
    cmd_t = normalize_time(cmd_t)
    enc_t = normalize_time(enc_t)



    print("======================")
    print("Loaded bag")
    print("======================")

    print(
        "Load cell shape:",
        load_data.shape
    )

    print(
        "Command shape:",
        cmd_data.shape
    )

    print(
        "Encoder shape:",
        enc_data.shape
    )



    # =================================================
    # Load Cell Plot
    # =================================================

    if len(load_data):

        #
        # load_data shape:
        #
        # samples x 20 x 6
        #
        # The DAQ duplicates the same wrench
        # 20 times.
        #
        # Pick one row.
        #

        wrench = load_data[:,0,:]


        names = [
            "Fx",
            "Fy",
            "Fz",
            "Tx",
            "Ty",
            "Tz"
        ]


        plt.figure(
            figsize=(14,7)
        )


        for i in range(6):

            plt.plot(
                load_t,
                wrench[:,i],
                label=names[i]
            )


        plt.title(
            "Load Cell Wrench"
        )

        plt.xlabel(
            "Time (s)"
        )

        plt.ylabel(
            "Measurement"
        )

        plt.grid()
        plt.legend()



    # =================================================
    # Servo command + encoder
    # =================================================

    if len(cmd_data) or len(enc_data):

        plt.figure(
            figsize=(14,7)
        )


        if len(cmd_data):

            cmd_data = np.array(
                cmd_data
            )


            for i in range(cmd_data.shape[1]):

                plt.plot(
                    cmd_t,
                    cmd_data[:,i],
                    label=f"Servo {i+1} command"
                )



        if len(enc_data):

            enc_data = np.array(
                enc_data
            )


            for i in range(enc_data.shape[1]):

                plt.plot(
                    enc_t,
                    enc_data[:,i],
                    "--",
                    label=f"Servo {i+1} encoder"
                )



        plt.title(
            "Servo Command vs Encoder"
        )

        plt.xlabel(
            "Time (s)"
        )

        plt.ylabel(
            "Position"
        )

        plt.grid()
        plt.legend()



    plt.show()



if __name__ == "__main__":
    main()