import biguasim
import json
import yaml

import numpy as np

from pathlib import Path

from biguasim_main.sensor_data_encode import encoders, multi_publisher_sensors

#TODO: Maybe add sensor data encode to this file

COMMAND_MAP = {
    'accel' : 6,
    'cmd_vel' : 3,
    'cmd_vel_yaw' : 4,
    'cmd_pos_yaw' : 4,
    'cmd_rudders_sterns_motor_speed' : 5,
    'cmd_depth_heading_rpm_surge' : 4
}

MOTOR_SPEEDS = {
    "BlueBoat": 2,
    "BlueROV2": 6,
    "BlueROVHeavy" : 8,
    "DjiMatrice" : 4
}

class BiguaSimInterface():
    '''
    Class for abstracting biguasim python interface
    Lists sensors and formats sensor data
    '''

    def __init__(self, scenario_path, init=True, node=None):
        """
        Initialize biguasim enviornment with a path to a json file for the scenario
        Create the vehicle object and the dynamics object
        """
        self.node = node
        scenario_path = Path(scenario_path)
        
        scenario = self.parse_scenario_yaml(scenario_path)

        #TODO: Make a parameter to use the system time
        self.system_time = True
        
        self.r2b = str.maketrans('_', '-')
        self.b2r = str.maketrans('-', '_')

        self.command = dict()

        #TODO: make sure dynamics sensor is enabled 
        if init:
            self.env = biguasim.make(scenario_cfg=scenario)
            self.scenario = self.env._scenario
            self.initialized = True
            self.sensors = self.create_sensor_list()
        else:
            self.scenario = scenario
            self.initialized = False

    def _get_agent_id(self, agent_name : str):
        split = agent_name.find('_')
        name = agent_name[:split]
        idx = int(agent_name[split+3:])
        return name, idx

    def _get_command_shape(self, agent_name, agent_type):
        control_abstraction = self.env._dynamics_dict[agent_name].control_abstraction
        if control_abstraction == 'cmd_motor_speeds':
            return MOTOR_SPEEDS[agent_type]
        return COMMAND_MAP[control_abstraction]


    def parse_scenario(self, path):
        file_path = Path(path)
        scenario = None

        with file_path.open() as params_file:
            scenario = json.load(params_file)

        return scenario

    
    def find_biguasim_scenario(self, yaml_content):
        """Recursively search for 'biguasim_scenario' in the YAML content."""
        if isinstance(yaml_content, dict):
            for key, value in yaml_content.items():
                if key == "biguasim_scenario":
                    return value
                else:
                    result = self.find_biguasim_scenario(value)
                    if result is not None:
                        return result
        elif isinstance(yaml_content, list):
            for item in yaml_content:
                result = self.find_biguasim_scenario(item)
                if result is not None:
                    return result
        return None

    def parse_scenario_yaml(self, scenario_path):
        with open(scenario_path, 'r') as file:
            yaml_content = yaml.safe_load(file)
        
        biguasim_scenario_yaml = self.find_biguasim_scenario(yaml_content)
        
        if biguasim_scenario_yaml is None:
            raise KeyError("Could not find 'biguasim_scenario' in the YAML file.")

        return biguasim_scenario_yaml

    def create_sensor_list(self):
        scenario = self.scenario

        if len(scenario["agents"]) > 1:
            self.multi_agent_scenario = True
            print("Give sensors unique names that are reported on multiple agents")
        else:
            self.multi_agent_scenario = False            

        sensors = []

        for agent in scenario["agents"]:
            #For each agent the control surface commands can be published 
            #TODO: Handle Multi agent scenario here
            agent_name = agent['agent_name'].translate(self.b2r)
            dynamics_name, _ = self._get_agent_id(agent_name)
            if not dynamics_name in self.command:
                batch_size = self.env._dynamics_dict[dynamics_name].batch_size
                cmd_shape = self._get_command_shape(dynamics_name, agent['agent_type'])

                if batch_size >1 :
                    self.command[dynamics_name] = np.zeros((batch_size, cmd_shape)).tolist()
                else:
                    self.command[dynamics_name] = np.zeros(cmd_shape).tolist()
            
            if "publish_commands" in agent and agent["publish_commands"]==True:
                encoder_class = encoders.get("ControlCommand")
                config = {}
                config['sensor_name'] = "ControlCommand"
                config['sensor_type'] = "ControlCommand"
                config['agent_name'] = agent_name
                config['state_name'] = 'ControlCommand'
                sensors.append(encoder_class(config))

            for sensor in agent["sensors"]:
                if sensor['ros_publish']:
                    sensor_type = sensor['sensor_type']
                    
                    if sensor_type in multi_publisher_sensors:
                        for suffix in multi_publisher_sensors[sensor_type]:
                            full_type = f"{sensor['sensor_type']}{suffix}"
                            sensor_name = sensor['sensor_name'] if 'sensor_name' in sensor else sensor['sensor_type']
                            full_name = f"{sensor_name}/{suffix}" if suffix != "" else sensor_name
                            
                            encoder_class = encoders.get(full_type)
                            sensor_copy = sensor.copy()  # Create a copy of the sensor dictionary
                            sensor_copy['sensor_name'] = full_name
                            sensor_copy['agent_name'] = agent_name
                            sensor_copy['state_name'] = sensor_name
                            sensors.append(encoder_class(sensor_copy))
                    else:
                        sensor_copy = sensor.copy()  # Create a copy of the sensor dictionary
                        sensor_copy['agent_name'] = agent_name
                        sensor_copy['state_name'] = sensor['sensor_name'] if 'sensor_name' in sensor else sensor['sensor_type']
                        encoder = encoders[sensor['sensor_type']]
                        sensors.append(encoder(sensor_copy))
        
        return sensors

    def publish_sensor_data(self, state : dict):
        self._state = state.copy()
        for sensor in self.sensors:
            try:
                agent_name, idx = sensor.agent_name.split('_id')
                msg = sensor.encode(state[agent_name][int(idx)][sensor.state_name])

                # Header
                if self.system_time:
                    msg.header.stamp = self.node.get_clock().now().to_msg()
                else:
                    msg.header.stamp.sec = int(state['t'])  # Set seconds part from state['t']
                    msg.header.stamp.nanosec = int((state['t'] - msg.header.stamp.sec) * 1e9)

                sensor.publisher.publish(msg)
            except KeyError:
                # Handle the case where sensor data is not available
                # self.get_logger().info(f"No data for sensor: {sensor['sensor_name']}")
                pass
            except Exception as e:
                # Handle other exceptions
                print(f"Error processing sensor: {sensor.name}, type: {sensor.type}, error: {str(e)}")

    def send_control_command(self, agent_name : str, command : list):
        
        dynamics_name, idx = self._get_agent_id(agent_name)
        
        if all(isinstance(x, list) for x in self.command[dynamics_name]):
            assert len(self.command[dynamics_name][idx]) == len(command)
            self.command[dynamics_name][idx] = command  

        else:
            assert len(self.command[dynamics_name]) == len(command)
            self.command[dynamics_name] = command      

    def tick(self):
        """
        Step the biguasim enviornment and the vehicle dynamics 
        Return the state
        """
        cmd = self.command
        if len(self.command) == 1:
            cmd = self.command[next(iter(self.command))]

        state = self.env.step(cmd)

        return state
    
    def get_scenario(self):
        if self.initialized:
            return self.env._scenario
        else:
            return self.scenario
    
    def get_tick_rate(self):
        if self.initialized:
            return self.env._ticks_per_sec
        else:
            if "ticks_per_sec" in self.scenario:
                return self.scenario['ticks_per_sec']
            else:
                ValueError('ticks_per_sec not specified in scenario')
    
    def get_frame_rate(self):
        if self.initialized:
            return self.env._frames_per_sec
        else:
            if "frames_per_sec" in self.scenario:
                return self.scenario['frames_per_sec']
            else:
                ValueError('frames_per_sec not specified in scenario')
    
    def get_period(self):
        return 1.0/self.get_tick_rate()
    
    def get_time_warp(self):
        #Check to make sure this is correct
        time_warp = self.get_frame_rate() / self.get_tick_rate()

        if time_warp <= 0:
            ValueError("frames_per_sec cannot be 0 for time warping. Set a value > 0 ")

        return time_warp

    def get_time_warp_period(self):
        return self.get_period() / self.get_time_warp() 
    



