import importlib.util
import os
import json


def read_json_from_package(package_name: str = 'disassembly_pipeline',
                     relative_json_path_within_package: str = 'config/object_pickup_parameters.json'):
    """
    Read and return a JSON from a relative location within a package.
    
    Example call:
    >>> config_json = load_config_json(package_name = 'disassembly_pipeline', relative_json_path_within_package = 'config/object_pickup_parameters.json')
    """

    out_json = None
    
    base_paths = importlib.util.find_spec(package_name).submodule_search_locations

    # In case of several base_paths, loop through them until finding the file
    for possible_path in base_paths:
        filepath = os.path.join(possible_path, relative_json_path_within_package)

        if not os.path.isfile(filepath):
            continue

        with open(filepath, 'r') as f:
            out_json = json.load(f)

    if out_json is None: raise ValueError(f"Did not find json config {relative_json_path_within_package} within package {package_name}")

    return out_json