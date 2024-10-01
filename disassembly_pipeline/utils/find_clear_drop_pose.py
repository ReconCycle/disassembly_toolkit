from typing import List, Union, Optional
from shapely.geometry import Polygon
import numpy as np
import time
import copy
import random

def get_random_drop_position(top_left, bottom_right):
    """ Get a random drop pose without checking for occupancy/occludedness."""
    x_min, y_max = top_left  # Top-left corner (X_min, Y_max)
    x_max, y_min = bottom_right  # Bottom-right corner (X_max, Y_min)

    x = random.uniform(x_min, x_max)  # Random X-coordinate within the range
    y = random.uniform(y_min, y_max)  # Random Y-coordinate within the range

    return x, y

def check_if_occluded(candidate_obb, other_obbs):
    #print(candidate_bbox)
    poly1 = Polygon(candidate_obb)
    for obb in other_obbs:
        poly2 = Polygon(obb)
        occluded = poly1.intersects(poly2)
        
        # Break out if any intersection detected, do not check subsequent elements.
        if occluded: return 1
    return 0

def find_drop_pose(input_obb: Union[List, np.array],
                   other_objects_bbox: Union[List, np.array], 
                   limit_bbox: Union[List, np.array],
                   sendTf = None,
                   parent_frame:Optional[str] = 'vision_table_zero',
                   n_pts_to_check_per_dim = 20,
                   verbose = 0
                  ):
    """
    Args:
    --------
    input_obb: oriented bounding box of object which we want to drop on table -> shape == (4,2)
    other_objects_bbox - oriented bounding boxes of objects within the workspace.
    limit_bbox: bbox within which candidate drop poses will be generated, in regards to parent frame.
    sendTf: sendTf object from disassembly_pipeline/utils/tf_manager
    parent_frame: parent frame is required if you want to publish transforms to ros to visualize drop pose
    n_pts_to_check_per_dim: How finely to divide the limit_bbox into points which will be checked - np.linspace num param.
    verbose: if verbose>1, print thing
    
    Example call:
    # Dimensions of object we want to drop on table, which we are currently holding with robot
    >>> dx = 0.1
    >>> dy = 0.1
    >>> input_obb = [[0,0],
                [dx,0],
                [dx, dy],
                [0, dy]]
    >>> other_detections = copy.deepcopy(VU.detections)
    >>> other_detections = [d.obb for d in other_detections]
    >>> find_drop_pose(input_obb= input_obb,
                       other_objects_bbox = other_detections, 
                       limit_bbox= [[0.15,0.45], [0.45,0.15]],
                       sendTf = sendTf,
                       parent_frame = 'vision_table_zero'
                      )
    """
    # Check input_obb and normalize it so it's centered ie lower left corner should have coords [0,0]
    assert len(input_obb) == 4
    input_obb = np.array(input_obb)
    input_obb[:, 0] -= np.min(input_obb[:,0])
    input_obb[:, 1] -= np.min(input_obb[:,1])
    x_dim = np.max(input_obb[:,0])
    y_dim = np.max(input_obb[:,1])
    if verbose >0: print("XDIM, YDIM", x_dim, y_dim)
        
    # Generate set of possible dropoff locations on table, limited by table size itself
    limit_bbox = np.array(limit_bbox)
    xmin = np.min(limit_bbox[:,0])
    xmax = np.max(limit_bbox[:,0])
    ymin = np.min(limit_bbox[:,1])
    ymax = np.max(limit_bbox[:,1])
    x = np.linspace(xmin, xmax, n_pts_to_check_per_dim)
    y = np.linspace(ymin, ymax, n_pts_to_check_per_dim)
    mesh = np.meshgrid(x,y)
    candidate_drop_poses = np.dstack(mesh) # (20,20, 2)
    sh1,sh2, sh3 = candidate_drop_poses.shape
    tuple_candidate_drop_poses = list(candidate_drop_poses.reshape(sh1*sh2, sh3)) # (400,2)
    
    # Trivial case: If no other objects, return random drop point
    if len(other_objects_bbox) == 0:
        return tuple_candidate_drop_poses[np.random.randint(0, len(tuple_candidate_drop_poses))] 
    
    final_result = None
    # TODO add timeout
    while (final_result is None) and (len(tuple_candidate_drop_poses)>1):
        # Randomly choose point
        random_pt_idx = np.random.randint(0, len(tuple_candidate_drop_poses))
        candidate_center_pt = tuple_candidate_drop_poses[random_pt_idx] # x,y
        tuple_candidate_drop_poses.pop(random_pt_idx) # Drop this point so we don't select it again
        
        # Generate obb from candidate_center_pt and the input_bbox
        moved_obb = copy.deepcopy(np.array(input_obb))
        moved_obb[:, 0] += candidate_center_pt[0] - x_dim/2
        moved_obb[:, 1] += candidate_center_pt[1] - y_dim/2
        
        is_occluded = check_if_occluded(candidate_obb = moved_obb,
                           other_obbs = other_objects_bbox)
        
        if (is_occluded) and (verbose>0) and (sendTf is not None) and (parent_frame is not None):
            sendTf(p = [candidate_center_pt[0], candidate_center_pt[1], 0], q = [0,0,0,1],
                          parent_frame = parent_frame,
                          child_frame = 'OCCL')

        if not is_occluded:
            final_result = [candidate_center_pt[0], candidate_center_pt[1], 0]
            if (sendTf is not None) and (parent_frame is not None):
                sendTf(p = final_result, q = [0,0,0,1],
                      parent_frame = parent_frame,
                      child_frame = 'TEST1')
            
            return final_result
    return "Failed, did not find candidate poses"

        
if __name__ == '__main__':   
    # Dimensions of object we want to drop on table, which we are currently holding with robot
    #dx = 0.14
    #dy = 0.14
    dx = 0.1
    dy = 0.1
    input_obb = [[0,0],
                [dx,0],
                [dx, dy],
                [0, dy]]
    other_detections = copy.deepcopy(VU.detections)
    other_detections = [d.obb for d in other_detections]

    for i in range(0,200):
        find_drop_pose(input_obb= input_obb,
                        other_objects_bbox = other_detections, 
                        limit_bbox= [[0.15,0.45], [0.45,0.15]],
                        sendTf = sendTf,
                        verbose = 0
                        )
        time.sleep(0.5)