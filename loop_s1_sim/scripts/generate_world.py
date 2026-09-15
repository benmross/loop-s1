#!/usr/bin/env python3
"""Write the Gazebo world for a course file, to look at or run by hand."""

import argparse
import os

from ament_index_python.packages import get_package_share_directory

from loop_s1_sim.course import load


def main():
    default = os.path.join(get_package_share_directory('loop_s1_sim'), 'config', 'course.yaml')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', help='where to write the .sdf')
    parser.add_argument('--course', default=default)
    args = parser.parse_args()
    with open(args.output, 'w') as f:
        f.write(load(args.course).world_sdf())
    print(args.output)


if __name__ == '__main__':
    main()
