'''plotcom.py

   Plot the /com recorded in the ROS2 bag.
'''

import rclpy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point

import glob, os, sys

from rosbag2_py                 import SequentialReader
from rosbag2_py._storage        import StorageOptions, ConverterOptions
from rclpy.serialization        import deserialize_message

from std_msgs.msg               import Float64


#
#  Plot the com Data
#
def plotcom(commsgs, t0, bagname):
    # Process the com messages.
    comx = np.array([msg.x for msg in commsgs])
    comy = np.array([msg.y for msg in commsgs])
    comz = np.array([msg.z for msg in commsgs])

    # Set up time, assuming a 100Hz rate.
    N  = len(comx)
    dt = 0.01
    t  = dt * np.linspace(0, N, N, endpoint=False)

    # Create a figure to plot com vs. t
    fig, axs = plt.subplots(3, 1)

    # Plot the data in the subplots.
    axs[0].plot(t, comx)
    axs[0].set(ylabel='X Position')
    axs[1].plot(t, comy)
    axs[1].set(ylabel='Y Position')
    axs[2].plot(t, comz)
    axs[2].set(ylabel='Z Position')

    # Connect the time.
    axs[1].set(xlabel='Time (sec)')
    axs[1].sharex(axs[0])
    axs[2].set(xlabel='Time (sec)')
    axs[2].sharex(axs[0])

    # Add the title and legend.
    axs[0].set(title="COM Data in '%s'" % bagname)

    # Draw grid lines and allow only "outside" ticks/labels in each subplot.
    for ax in axs.flat:
        ax.grid()
        ax.label_outer()


#
#  Main Code
#
def main():
    # Grab the arguments.
    bagname   = 'latest' if len(sys.argv) < 2 else sys.argv[1]

    # Check for the latest ROS bag:
    if bagname == 'latest':
        # Report.
        print("Looking for latest ROS bag...")

        # Look at all bags, making sure we have at least one!
        dbfiles = glob.glob('*/*.db3')
        if not dbfiles:
            raise FileNoFoundError('Unable to find a ROS2 bag')

        # Grab the modification times and the index of the newest.
        dbtimes = [os.path.getmtime(dbfile) for dbfile in dbfiles]
        i = dbtimes.index(max(dbtimes))

        # Select the newest.
        bagname = os.path.dirname(dbfiles[i])

    # Report.
    print("Reading ROS bag '%s'"  % bagname)


    # Set up the BAG reader.
    reader = SequentialReader()
    try:
        reader.open(StorageOptions(uri=bagname, storage_id='sqlite3'),
                    ConverterOptions('', ''))
    except Exception as e:
        print("Unable to read the ROS bag '%s'!" % bagname)
        print("Does it exist and WAS THE RECORDING Ctrl-c KILLED?")
        raise OSError("Error reading bag - did recording end?") from None

    # Get the starting time.
    t0 = reader.get_metadata().starting_time.nanoseconds * 1e-9 - 0.01

    # Get the topics and types:
    print("The bag contain message for:")
    for x in reader.get_all_topics_and_types():
        print("  topic %-20s of type %s" % (x.name, x.type))


    # Pull out the relevant messages.
    commsgs = []
    while reader.has_next():
        # Grab a message.
        (topic, rawdata, timestamp) = reader.read_next()

        # Pull out the deserialized message.
        if   topic == '/com':
            commsgs.append(deserialize_message(rawdata, Point))


    # Process the com number.
    if commsgs:
        print("Plotting com data...")
        plotcom(commsgs, t0, bagname)
    else:
        raise ValueError("No com data!")

    # Show
    plt.show()


#
#   Run the main code.
#
if __name__ == "__main__":
    main()