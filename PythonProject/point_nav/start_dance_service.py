#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import errno


_LOCK_FILE_PATH = "/tmp/dance_service.lock"
_lock_file_handle = None


def acquire_single_instance_lock():
    """
    Ensure this launcher runs as a single instance using an OS file lock.
    Returns True if lock acquired, False if another instance is running.
    """
    global _lock_file_handle
    try:
        _lock_file_handle = open(_LOCK_FILE_PATH, "w")
        try:
            import fcntl
            fcntl.lockf(_lock_file_handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return False
        except OSError as e:
            # If cannot lock for other reasons, continue without lock
            print(f"Warning: failed to lock {_LOCK_FILE_PATH}: {e}")
        _lock_file_handle.write(str(os.getpid()))
        _lock_file_handle.flush()
        return True
    except Exception as e:
        print(f"Warning: cannot create lock file {_LOCK_FILE_PATH}: {e}")
        return True


def ensure_sys_path():
    """
    Ensure required paths are in sys.path to import dance_service and g1_client_cwk.
    Does not modify g1_client_cwk.py; only adjusts import search path at runtime.
    """
    paths = [
        "/home/unitree/HongTu/PythonProject/point_nav",
        "/home/unitree/unitree_sdk2_python/example/g1/high_level",
    ]
    for p in paths:
        if p not in sys.path:
            sys.path.append(p)


def main():
    # Single instance guard
    if not acquire_single_instance_lock():
        print("Another start_dance_service instance is already running. Exit.")
        return

    ensure_sys_path()

    # Lazy import to ensure paths have been injected
    try:
        import rospy
    except Exception as e:
        print(f"Failed to import rospy: {e}")
        sys.exit(1)

    # Friendly tip for ROS env
    ros_master = os.environ.get("ROS_MASTER_URI", "")
    if not ros_master:
        print("Warning: ROS_MASTER_URI not set. If using external ROS master, export it first.")

    try:
        from dance_service import DanceService
    except Exception as e:
        print(f"Failed to import DanceService: {e}")
        sys.exit(1)

    try:
        service = DanceService()
        if not rospy.core.is_initialized():
            # Fallback: normally DanceService.__init__ already init_node
            rospy.init_node("dance_service")
        print("start_dance_service: DanceService started. Entering ROS spin loop...")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("KeyboardInterrupt: exiting")
    except rospy.ROSInterruptException:
        print("ROSInterruptException: exiting")
    except Exception as e:
        print(f"Failed to run DanceService: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()


