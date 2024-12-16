from typing import List
import warnings
from termcolor import colored


class ErrorValidator:
    def __init__(self):
        self.nameSpacesAndExecutables: List[tuple] = []

    def checkNameSpace(self, namespace: str, executable: str, enable):
        if enable != 'true':
            return
        if (namespace, executable) in self.nameSpacesAndExecutables:
            raise ValueError(colored(f"THE NAMESPACE '{namespace}' AND EXECUTABLE '{executable}' ARE ALREADY DEFINED", 'red'))
        self.nameSpacesAndExecutables.append((namespace, executable))

    @staticmethod
    def validateLaunchConfiguration(launchConfiguration: dict, parent: str):
        requiredKeys = ['executable', 'package_name', 'enable', 'parameters']
        for key in requiredKeys:
            if key not in launchConfiguration:
                raise ValueError(colored(f"THE PARAM '{key}' IS MISSING IN '{parent}' LAUNCH CONFIGURATION", 'red'))
        if not isinstance(launchConfiguration['parameters'], dict):
            raise TypeError(colored("PARAMETERS SHOULD BE A DICTIONARY", 'red'))
        
        # if 'name' not in launchConfiguration['parameters']:
        #     raise ValueError(colored(f"THE 'parameters' DICTIONARY MUST CONTAIN THE 'name' KEY IN '{parent}' LAUNCH CONFIGURATION", 'red'))
        # if 'namespace' not in launchConfiguration['parameters']:
        #     raise ValueError(colored(f"THE 'parameters' DICTIONARY MUST CONTAIN THE 'namespace' KEY IN '{parent}' LAUNCH CONFIGURATION", 'red'))
        
        if not launchConfiguration['executable'].endswith('.launch.py'):
            raise ValueError(colored(f"THE LAUNCH FILE '{launchConfiguration['executable']}' MUST HAVE THE SUFFIX '.LAUNCH.PY'", 'red'))

        for key, value in launchConfiguration['parameters'].items():
            if value is None:
                warnings.warn(colored(f"THE PARAMETER '{key}' IN '{parent}' LAUNCH CONFIGURATION CANNOT BE NONE", 'yellow'), UserWarning)

    @staticmethod
    def checkLaunchListNotEmpty(launchList: List):
        if len(launchList) == 0:
            raise RuntimeError(colored("NO LAUNCH CONFIGURATIONS ARE ENABLED. ENSURE THAT THE 'ENABLE' PARAMETER IS SET TO 'TRUE' AS A STRING IN THE CONFIGURATION FILE.", 'red'))
