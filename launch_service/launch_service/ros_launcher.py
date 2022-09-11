from enum import Enum
import threading, os, signal

from ament_index_python.packages import PackageNotFoundError

from rclpy import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import launch
import rclpy
from ros2launch.api import get_share_file_path_from_package
from ros2launch.api import is_launch_file
from ros2launch.api import launch_a_launch_file
from ros2launch.api import MultipleLaunchFilesError

from launch_msgs.msg import LaunchID
from launch_msgs.srv import ListLaunch, StartLaunch, StopLaunch

def get_host():
    return os.uname()[1]

class LaunchStatus(Enum):
    UNKNOWN = 0
    RUNNING = 1

    EXITED = 2
    KILLED = 3
    ERRORED = 4

class LaunchRunner:
    def __init__(self, path) -> None:
        self.return_code = 0
        self.path = path
        self.status_val = LaunchStatus.UNKNOWN

        # create and start the thread
        self.thread = threading.Thread(target = self.run)
        self.thread.start()

    def kill(self):
        # used to stop this thread by sending a sigint
        self.status_val = LaunchStatus.KILLED
        signal.pthread_kill(self.thread.ident, signal.SIGINT)

    def status(self):
        # gets the status of the thread as well as a return code if stopped
        return (self.status_val, self.return_code)

    def run(self):
        self.status_val = LaunchStatus.RUNNING

        self.return_code = launch_a_launch_file(
            launch_file_path = self.path,
            noninteractive=True
        )

        # something has interrupted it. Record the return code, and stop the context
        if(self.status_val != LaunchStatus.KILLED):
            if(self.return_code != 0):
                self.status_val = LaunchStatus.ERRORED

            else:
                self.status_val = LaunchStatus.EXITED



class LaunchNode(Node):
    def __init__(self, hostname) -> None:
        super().__init__(f'{hostname}_launch_srv')

        # callback groups for the executor
        self.service_group = ReentrantCallbackGroup()
        self.timer_group = ReentrantCallbackGroup()

        # service servers for executing callbacks
        self.start_srv = self.create_service(StartLaunch, 'start_launch', self.start_cb, callback_group=self.service_group)
        self.list_srv = self.create_service(ListLaunch, 'list_launch', self.list_cb, callback_group=self.service_group)
        self.stop_srv = self.create_service(StopLaunch, 'stop_launch', self.stop_cb, callback_group=self.service_group)

        # timer for checking status of each internal thread
        # we can check 
        self.stat_timer = self.create_timer(0.1, self.check_cb, callback_group=self.timer_group)

        # current launch ID to be used on the next startup
        self.start_id = 0

        # map for managing internal threads
        # structure has launch ID # as the key, and contains a tuple of LaunchThread and LaunchID
        self.launch_map = {}

    def start_cb(self, req, resp):
        # at this point req contains two valid fields
        launch_file = str(req.launch_file)
        launch_pkg = str(req.package)

        launch_path = launch_file
        try:
            if not os.path.isfile(launch_file):
                # we were given a package and a file name
                # first we need to parse the location of the file
                launch_path = get_share_file_path_from_package(
                        package_name=launch_pkg,
                        file_name=launch_file)
            
            if not is_launch_file(launch_path):
                raise RuntimeError(f"launch file {launch_path} is not valid!")

        except PackageNotFoundError as exc:
            resp.started = False
            resp.error = "Package '{}' not found: {}".format(launch_pkg, exc)
            return 
        except (FileNotFoundError, MultipleLaunchFilesError, RuntimeError) as exc:
            resp.started = False
            resp.error = str(exc)
            return

        # build the LaunchID
        launch_id = LaunchID()
        launch_id.launch_id = self.start_id
        launch_id.status = LaunchID.RUNNING
        launch_id.package = launch_pkg
        launch_id.launch_file = launch_file
        self.start_id += 1

        # Build and start the launch context
        launch_context = LaunchRunner(launch_path)

        # fill out the launch ID field in the response
        resp.formed_launch = launch_id

        # add the entry in the dictonary
        self.launch_map[launch_id.launch_id] = (
            launch_id,
            launch_context
        )

    def list_cb(self, req, resp):
        launches = []
        # work through the list of launches to show 
        for key in self.launch_map.keys():
            launch_id = self.launch_map[key](0)

            # filter it such that unknown in the request gives all, or another gives only that type
            if(req.status == LaunchID.UNKNOWN or req.status == launch_id.status):
                launches.append(launch_id)
        
        resp.launches = launches

    def stop_cb(self, req, resp):
        # check to see if the launch ID exists and is alive
        pass

    def check_cb(self, req, resp):
        # check the status of every running thread
        pass

    def closeout(self):
        # go through each thread and shut them down if they were in a running state

        pass


def main():
    hostname = get_host()

    node = LaunchNode(hostname=hostname)

    # create the executro with 2 threads. one for each group
    exec = MultiThreadedExecutor(2)

    # spin the ros threads
    rclpy.spin(node, executor=exec)

    # we have been intterupted, shut down all the running threads
    node.closeout()

    # when interrupted, shut down cleanly
    node.destroy_node()
    rclpy.shutdown()
    




if __name__ == '__main__':
    main()
