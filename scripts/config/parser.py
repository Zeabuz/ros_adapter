import rospy
import roslib
import rosparam

def parse_scenario_config(config_dir_path, scenario_id):
    scenario_params = rosparam.load_file(config_dir_path + "scenarios.yaml")
    for params, namespace in scenario_params:
        rosparam.upload_params(namespace, params)

    simulation_params = rospy.get_param('/')
    config_files = simulation_params["scenario" + scenario_id]["config_files"]

    for config_file in config_files:
        paramlist = rosparam.load_file(config_dir_path + config_file)
        for params, namespace in paramlist:
            rosparam.upload_params(namespace, params)

    simulation_params = rospy.get_param('/')

    return simulation_params
