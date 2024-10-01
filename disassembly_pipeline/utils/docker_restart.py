import subprocess

def restart_softclaw_docker(command = "docker restart ifam2024-softclaw_client-1",
                            robot_name = 'panda_2',
                            password_env_variable = 'BAMBUS_PASS'):
    
    password = os.environ[password_env_variable]

    robot_name_to_computer_username_and_ip_dict = {'panda_1':  "panda@10.20.0.21",
                                                   'panda_2': "panda@10.20.0.22"}

    if robot_name not in robot_name_to_computer_username_and_ip_dict.keys():
        raise ValueError("Wrong robot name, computer username and IP not set for this robot.")

    user_and_ip = robot_name_to_computer_username_and_ip_dict[robot_name]
    subprocess.run(["sshpass", "-p", password, "ssh", user_and_ip, command])

def restart_vise_docker(command = "docker restart raspi-vise-pneumatics",
                        password_env_variable = 'RASPI_CLAMP_PASS'):
    
    password = os.environ[password_env_variable]

    user_and_ip = "pi@raspi-clamp-block.local"
    subprocess.run(["sshpass", "-p", password, "ssh", user_and_ip, command])

def restart_cutter_docker(command = "docker restart raspi-cutter-pneumatics",
                        password_env_variable = 'RASPI_CUTTER_PASS'):
    
    password = os.environ[password_env_variable]

    user_and_ip = "pi@raspi-cutter-block.local"
    subprocess.run(["sshpass", "-p", password, "ssh", user_and_ip, command])
