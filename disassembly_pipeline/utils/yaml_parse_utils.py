import numpy as np
import yaml


def parse_yaml_to_get_q(filename, out='concatenated'):
    """ Function to parse YAML which was saved by FrankaDesk.
    Args:
    filename : str
        name of file (string)

    out : if 'concatenated', you get an array of q x 7
        if 'block', you get an array of n x q x 7
    
    Returns:
    q_blocks: array of (q x 7) or (n x q x 7)"""
    
    assert out in ['blocks', 'concatenated']

    with open(filename, 'r') as stream:
        data = yaml.safe_load(stream)

    q_list = data['parameter']['poses']
    q_arr = np.array(q_list)

    q_blocks = []
    n_of_moves=[]

    for i in range(len(q_arr)):
    #for i in range(1):
        n_of_moves.append(len(q_arr[i]['relative_trajectories'][0]))
        #rint(arr[i]['relative_trajectories'])
        print(n_of_moves[i])
        bl = []
        for j in range(0,n_of_moves[i]):
            print(np.array((q_arr[i]['relative_trajectories'][0][j]['joint_angles'])))
            bl.append(q_arr[i]['relative_trajectories'][0][j]['joint_angles'])

        q_blocks.append(bl)
    
    if out == 'concatenated':
        all_joint_positions = np.vstack(q_blocks)
        return all_joint_positions
    elif out =='blocks':
        return q_blocks
