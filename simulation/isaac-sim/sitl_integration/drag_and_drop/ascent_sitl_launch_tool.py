"""

"""
# System tools used to launch the sitl process in the brackground
import os
import tempfile
import subprocess


class AscentSitlLaunchTool:
    """
    A class that manages the start/stop of a sitl process. It requires only the path to the sitl installation (assuming that
    sitl was already built with 'make sitl_sitl_default none'), the vehicle id and the vehicle model. 
    """

    def __init__(self, sitl_dir, vehicle_id: int = 0):
        """Construct the sitlLaunchTool object

        Args:
            sitl_dir (str): A string with the path to the sitl-Autopilot directory
            vehicle_id (int): The ID of the vehicle. Defaults to 0.
            sitl_model (str): The vehicle model. Defaults to "iris".
        """


        # Attribute that will hold the sitl process once it is running
        self.sitl_process = None

        # The vehicle id (used for the mavlink port open in the system)
        self.vehicle_id = vehicle_id
        
        self.base_port = 5760 + self.vehicle_id*10
        self.ascent_sitl_port = 14552 + self.vehicle_id*10
        self.isaac_sim_port = 14553 + self.vehicle_id*10
        self.autonomy_stack_port = 14554 + self.vehicle_id*10
        self.mavros_launch_port = 14555 + self.vehicle_id*10

        # Configurations to whether autostart sitl (SITL) automatically or have the user launch it manually on another
        # terminal
        self.sitl_dir = sitl_dir
        #self.sitl_script = self.sitl_dir + "/launch_ascent_sitl.bash"
        self.sitl_script = '/usr/bin/bash ' + self.sitl_dir + '/ascent_sitl_tmux.bash ' + \
            str(self.base_port) + ' ' + str(self.ascent_sitl_port) + ' ' + \
            str(self.isaac_sim_port) + ' ' + str(self.autonomy_stack_port) + ' ' + str(self.mavros_launch_port) + \
            ' ' + str(self.vehicle_id)

        # Create a temporary filesystem for sitl to write data to/from (and modify the origin rcS files)
        self.root_fs = tempfile.TemporaryDirectory()

        # Set the environement variables 
        self.environment = os.environ
        # self.environment["sitl_SIM_MODEL"] = sitl_model

    def get_dronekit_address(self):
        return '127.0.0.1:' + str(self.isaac_sim_port)

    def launch(self):
        """
        Method that will launch a sitl instance with the specified configuration
        """
        self.sitl_process = subprocess.Popen(self.sitl_script, shell=True)
        '''
        self.sitl_process = subprocess.Popen(
            [
                "bash",
                self.sitl_dir + '/ascent_sitl_tmux.bash'
            ],
            cwd=self.root_fs.name,
            shell=False,
            env=self.environment,
        )
        #'''

    def kill_sitl(self):
        """
        Method that will kill a sitl instance with the specified configuration
        """
        if self.sitl_process is not None:
            self.sitl_process.kill()
            self.sitl_process = None

    def __del__(self):
        """
        If the sitl process is still running when the sitl launch tool object is whiped from memory, then make sure
        we kill the sitl instance so we don't end up with hanged sitl instances
        """

        # Make sure the sitl process gets killed
        if self.sitl_process:
            self.kill_sitl()

        # Make sure we clean the temporary filesystem used for the simulation
        self.root_fs.cleanup()


# ---- Code used for debugging the sitl launch tool ----
def main():

    script_dir = os.path.dirname(os.path.realpath(__file__))
    sitl_tool = AscentSitlLaunchTool(script_dir)
    sitl_tool.launch()

    import time

    time.sleep(60)


if __name__ == "__main__":
    main()
