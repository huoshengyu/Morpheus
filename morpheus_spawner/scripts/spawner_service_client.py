#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy

from morpheus_spawner.srv import *

def spawner_service_client(preset_name = "dragon"):
    # Make sure the spawner service is running first
    rospy.wait_for_service("spawner")
    # Attempt to call the spawner service and print whether it succeeded
    try:
        spawner = rospy.ServiceProxy("spawner", SpawnerService)
        req = SpawnerServiceRequest(preset_name)
        res = spawner(req)
        return res.success
    # Notify and exit if the call fails
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    return

def usage():
    return "%s [preset_name]"%sys.argv[0]

if __name__ == "__main__":
    # Check command line argument count, retrieve name of collision_object.yaml entry
    if len(sys.argv) == 2:
        preset_name = str(sys.argv[1])
    # If command line args incorrect, print a reminder and exit
    else:
        print(usage())
        sys.exit(1)
    # Print input
    print("Requesting preset named %s"%(preset_name))
    # Call & print output
    success = spawner_service_client(preset_name)
    print("Success: %s"%(success))