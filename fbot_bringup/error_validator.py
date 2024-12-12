from typing import List


class ErrorValidator:
    def __init__(self):
        self.nameSpacesAndExecutables: List[tuple] = []

    def checkNameSpace(self, namespace: str, executable: str, enable):
        if enable != 'true':
            return
        if (namespace, executable) in self.nameSpacesAndExecutables:
            raise ValueError(f"\033[91mThe namespace '{namespace}' and executable '{executable}' are already defined\033[0m")
        self.nameSpacesAndExecutables.append((namespace, executable))

    @staticmethod
    def validateLaunchConfiguration(launchConfiguration: dict, parent: str):
        requiredKeys = ['executable', 'package_name', 'enable', 'parameters']
        for key in requiredKeys:
            if key not in launchConfiguration:
                raise ValueError(f"\033[91mThe param '{key}' is missing in '{parent}' launch configuration\033[0m")
        if not isinstance(launchConfiguration['parameters'], dict):
            raise TypeError("\033[91mParameters should be a dictionary\033[0m")
        
        if 'name' not in launchConfiguration['parameters']:
            raise ValueError(f"\033[91mThe 'parameters' dictionary must contain the 'name' key in '{parent}' launch configuration\033[0m")
        if 'namespace' not in launchConfiguration['parameters']:
            raise ValueError(f"\033[91mThe 'parameters' dictionary must contain the 'namespace' key in '{parent}' launch configuration\033[0m")
        
        if not launchConfiguration['executable'].endswith('.launch.py'):
            raise ValueError(f"\033[91mThe launch file '{launchConfiguration['executable']}' must have the suffix '.launch.py'\033[0m")

    @staticmethod
    def checkLaunchListNotEmpty(launchList: List):
        if len(launchList) == 0:
            raise RuntimeError("\033[91mNo launch configurations are enabled. Ensure that the 'enable' parameter is set to 'true' as a string in the configuration file.\033[0m")
