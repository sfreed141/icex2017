#!/usr/bin/python
import sys
import math

def waypoints_from_file(filename):
    """
    This function does cool stuff.
    """
    waypoints = []
    with open(filename, 'r') as infile:
        # Extract lines corresponding to the waypoints
        in_waypoint_section = False
        for line in infile:
            if line.rstrip() == "START":
                in_waypoint_section = True
                continue
            if in_waypoint_section:
                if line.rstrip() == "END":
                    break
                fields = line.split(';')
                wp_num = int(fields[0])
                coord = (float(fields[1]), float(fields[2]))
                waypoints.append((wp_num, coord))
    return waypoints

def print_waypoints(waypoints):
    for wp in waypoints:
        print("{: 3d}: ".format(wp[0]), end='')
        print("{: 12.8f}, {: 12.8f}".format(*wp[1]), end='')
        print(" -> {: 3d}: ".format(wp[0]), end='')
        print("{: 3}° {: 7.3f}', {: 3}° {: 7.3f}'".format(*convert_coords(wp[1])))

def convert_coords(data):
    (lat, long) = data
    return (math.floor(lat), 60 * (lat % 1), math.floor(long), 60 * (long % 1))

def waypoints_from_cla():
    waypoints = []
    coords = [float(i) for i in sys.argv[1:]]
    for i, coord in enumerate(zip(coords[0::2], coords[1::2])):
        waypoints.append((i, coord))
    return waypoints

def main():
    waypoints = []
    if (sys.argv[1] == "-m"):
        print("Loading waypoints from file {}".format(sys.argv[2]))
        waypoints = waypoints_from_file(sys.argv[2])
    else:
        waypoints = waypoints_from_cla()
#    for wp in waypoints:
#        print("{}, {}".format(*wp))
    print_waypoints(waypoints)

if __name__ == '__main__':
    main()

