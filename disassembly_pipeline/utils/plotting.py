import numpy as np
import matplotlib.pyplot as plt
import os


def plot_q_or_v_or_a(al, form = 'q'):
    assert form in ['x', 'q']
    if form =='x':
        labels = ['x', 'y', 'z', 'w' ,'xq', 'yq','zq']
    else:
        labels = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'j7']
    
    colors = []
    print("Input array shape: ", al.shape)
    
    for i in range(0,al.shape[1]):
        plt.plot(al[:,i], label=labels[i])

    plt.legend(loc='upper right')
    
def plot_q_trajectory(traj, plot_vel = 0):
    """r = panda_ros
    plot r.recorded_jmove_trajectories"""
    
    qs = traj[0]
    qdots = traj[1]
    if plot_vel:
        plt.plot(qdots)
    else:
        plt.plot(qs)
    0
    
def plot_joint_trajectory_goal(traj, plot_vel=0):
    """
    Plot the goal sent to JointPositionTrajectory controller
    r = panda_ros
    plot r.recorded_joint_trajectories"""
    pts = traj.trajectory.points

    
    qs = []
    qdots = []
    ts = []
    
    for i in pts:
        qs.append(i.positions)
        qdots.append(i.velocities)
        
        secs = i.time_from_start.secs
        nsecs = i.time_from_start.nsecs
        ts.append(secs + nsecs*10**-9)
    qs = np.array(qs)
    qdots = np.array(qdots)
    ts = np.array(ts)
    #qdots = traj.trajectory.points.velocities
    idx_end = -1
    #idx_end = 10
    
    if plot_vel:
        plt.plot(ts[0:idx_end], qdots[0:idx_end,:])
        plt.ylabel("qdot [rad/s]")
    else:
        plt.plot(ts[0:idx_end], qs[0:idx_end])
        plt.ylabel('q [rad]')
        
    plt.grid()
    plt.xlabel('n')

def plot_x_trajectory(traj, plot_vel=0):
    """r = panda_ros
    plot r.recorded_cmove_trajectories"""
    xs = traj[0]
    xdots = traj[1]
    if plot_vel:
        plt.plot(xdots)
    else:
        plt.plot(xs)
        
def get_time_breakpoints(input_timesamples, min_diff = 0.1):
    """For an input array of timesamples, get the breakpoints where subsequent timesamples have a difference more than
    min_diff.
    Basically allows you to partition different movements in the recorded states.
    Args:
    min_diff [s]- minimal diff between subsequent samples that triggers the breakpoint detection"""
    # Separate stuff based on difference between ts
    breakpoints = []

    for i in range(1,len(ts)):
        if (ts[i] - ts[i-1]) > min_diff:
            breakpoints.append(i-1)
    print("Breakpoints:", breakpoints)
    return breakpoints

def parse_recording(robot = None, recording = None):
    
    ts = []
    qs = []
    xs = []
    FTs = []
    
    commanded_x = []
    commanded_q = []
    
    actual_x = []
    actual_q = []

    initial_timestamp = recording[0][0].header.stamp.secs + recording[0][0].header.stamp.nsecs * 10**-9

    for i in recording:
        robot_states = i[0]
        command_int = i[1]
        actual_int = i[2]
        
        ts.append(robot_states.header.stamp.secs + robot_states.header.stamp.nsecs * 10**-9)
        #print()
        qs.append(robot_states.q)

        x, _ = robot.kinmodel(robot_states.q)
        xs.append(x)

        FTs.append(robot_states.K_F_ext_hat_K)
        
        commanded_x.append(command_int.x)
        commanded_q.append(command_int.q)
        
        actual_x.append(actual_int.x)
        actual_q.append(actual_int.q)

    ts = np.array(ts)
    ts[:] = ts[:] - ts[0] # Start time at 0
    
    qs = np.array(qs)
    xs = np.array(xs)
    FTs = np.array(FTs)
    
    commanded_x = np.array(commanded_x)
    commanded_q = np.array(commanded_q)
    
    actual_x = np.array(actual_x)
    actual_q = np.array(actual_q)
    
    return ts, qs, xs, FTs, commanded_x, commanded_q, actual_x, actual_q

def save_recording(array, name = 'levering', hca_type = 'None', folder = 'disassembly_pipeline/recordings'):
    
    fn = "{0}_{1}".format(name, hca_type)

    cur_files = os.listdir(folder)
    
    max_found_index = 0
    for i in range(0,len(cur_files)):
        f = cur_files[i]
        
        if fn in f:
            print("Found file", f)
            o = f.split('_')[-1]
            last_idx = int(o[:-4])
            if last_idx > max_found_index:
                max_found_index = last_idx
        
    
    fn = folder + '/' + fn + "_%d.npy"%(max_found_index+1)
    
    with open(fn, 'wb') as f:
        np.save(file =f, arr = array)
        print("Saving to file %s"%fn)
        
def load_recording(name = 'levering_adapted_kalo.npy', folder = 'disassembly_pipeline/recordings'):
    fn = folder+'/'+name
    print("Opening file %s"%fn)
    with open(fn, 'rb') as f:
        a = np.load(f, allow_pickle = True)
    return a
    
def main():
    0
    #plot_joint_trajectory_goal(r.recorded_joint_trajectories[-1], plot_vel = 1)
    #plot_q_trajectory(r.recorded_jmove_trajectories[-1], plot_vel=0)
    #plot_x_trajectory(r.recorded_cmove_trajectories[-4], plot_vel=1)
    #plot_q_or_v_or_a(a[:10, :])
    
if __name__ == '__main__':
    main()
    

