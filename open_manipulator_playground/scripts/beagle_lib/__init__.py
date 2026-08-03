"""Beagle driving primitives, vendored from the Beagle Day2/Day3 course package.

Source: Beagle_Day2_Day3_Student_Labs, MIT licensed - see LICENSE.txt in this
directory.  Only the modules the stock relay mission needs were copied:

    geometry.py  pose, odometry integration, wheel/twist conversions
    lidar.py     scan cleaning, sector and cardinal distances, opening detection
    robot.py     SafeBeagle wrapper and the MockBeagle 2D simulator
    motion.py    gyro-integrating turns and lidar centring
    control.py   hysteresis, watchdog, wall following

These files are kept unmodified so they can be re-synced from the course package
when it is updated.  Anything specific to this project belongs in the nodes that
import them, not in here.
"""
