 
def perform_DMP(robot_obj,
               file_to_load = '/ros_ws/src/disassembly_pipeline/disassembly_pipeline/dmp/vise_to_above_table.txt',
               final_goal = None,
               space = 'joint'):
    
    assert space in ['cartesian', 'joint']
    
    from robotblockset_python.paths.trajectories import gradientPath, uniqueQuaternion, gradientPath, gradientCartesianPath
    from disassembly_pipeline.dmp.dmp import DMP, CartDMP
    from robotblockset_python.panda_ros import execute_arbitrary_joint_trajectory, execute_arbitrary_cartesian_trajectory
    
    r = robot_obj

    ### Getting the DMP trajectory
    qi = np.loadtxt(file_to_load,
                    delimiter = ';', dtype = np.float32)#[1:,:]

    #qq = np.array(qi)[1:,:] # wtf, why the 1 ?
    
    qq = qi
    assert qq.shape[1] == r.nj
    
    n_dodatnih_tock = int(qq.shape[0] * 0.07)
    new_array = np.zeros((qq.shape[0] + n_dodatnih_tock, r.nj))
    new_array[:] = np.nan
    new_array[0:qq.shape[0],:]=qq
    new_array[qq.shape[0]:,:] =qq[-1,:]
    
    qq = new_array
    
    # Tukej zdej vzames 2% dolzine trajektorije.
    # Na konc trajektorije dodas kopijo zadnje tocke vseh jointov
    if space == 'joint':
        
        dmp_obj = DMP(pos_data = qq, time = r.tsamp, num_weights = 8)

        #if final_goal is not None:
            #dmp_obj.goal = final_goal

        decoded = dmp_obj.decode()
        decoded_q = np.array(decoded[0][1:])
        decoded_t = np.array(decoded[1][1:])

        decoded_qdot = gradientPath(decoded_q, decoded_t)
        #return decoded_q, decoded_qdot, decoded_t
        assert decoded_q.shape[1] == r.nj
        assert decoded_t.shape[0] == decoded_q.shape[0]

        #plt.plot(decoded_q)
        #plt.plot(decoded_qdot)
        ### END Getting the DMP trajectory
        
        if r._control_strategy != 'JointPositionTrajectory':
            r.Switch_controller(start_controller =  'JointPositionTrajectory')
        
        r.error_recovery()
        # If we are far away from initial q, move to it
        if np.sum(np.abs((np.array(r._actual_int.q) - decoded_q[0,:]))) > 0.1:
            print("perform_DMP: {} is not at expected initial position, therefore moving to it".format(r.Name))
            r.JMove(decoded_q[0,:],2, qdot_max_factor = 0.2, qddot_max_factor = 0.2)

        execute_arbitrary_joint_trajectory(robot_obj = r,
                                           qs = decoded_q,
                                           qdots = decoded_qdot,
                                           times = decoded_t)
    
    elif space == 'cartesian':
        
        #if r._control_strategy != 'CartesianImpedance':
        #    r.Switch_controller(start_controller =  'CartesianImpedance')
        print("Entering cartesian DMP")
        xi = []
        for i in range(0,qq.shape[0]):
            xn = r.kinmodel(qq[i, :])[0]
            xi.append(xn)
        xi = np.array(xi)
        assert xi.shape[1] == 7
        quaternions = xi[:,3:]
        quaternions = uniqueQuaternion(quaternions)
        xi[:,3:] = quaternions
        
        dmp_obj = CartDMP(traj_samples = xi, time = r.tsamp, num_weights = 8)
        
        pos_traj, rot_traj, times = dmp_obj.decode()
        pos_traj = pos_traj[1:,:]
        rot_traj = rot_traj[1:, :]
        times = times[1:]
        xi = np.hstack([pos_traj,rot_traj])
        
        xdots = gradientCartesianPath(xi,times)
        
        execute_arbitrary_cartesian_trajectory(robot_obj = r,
                                           xs = xi,
                                           xdots = xdots,
                                           times = times)
        return xi, xdots, times
            
    #return decoded_q, decoded_qdot, decoded_t