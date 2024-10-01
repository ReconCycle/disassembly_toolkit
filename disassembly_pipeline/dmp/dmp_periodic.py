from .dmp import DMP

import numpy as np

class PeriodicDMP(DMP):
    """PeriodicDMP Class.

    This is the class that implements the Periodic Dynamic Movement Primitives - PDMPs.
    The class supports both Encoding and Decoding of trajectories.

    Attributes
    ----------
    num_weights : int
        Number of DMP weights.
    a_x : float
        The a_x gain of the DMP.
    a_z : float
        The a_z gain of the DMP. (damping)
    b_z : float
        The b_z gain of the DMP. (stiffness of the position-to-goal spring)
    c : array like
        The width of the Gaussian bell curves.
    y0 : 1 dimensional array
        The initial position of the DMP
    goal : 1 dimensional array
        The final position of the DMP
    sigma : array of floats
        The values of the Gaussian bell curves.
    num_dof : int
        The number of weights used for estimating the DMP.
    weights_pos : array of floats
        The calulated weights of the DMP.

    Parameters
    ----------
    pos_data : np.array or float list
        Data points of the trajectory to encode.
    vel_data : np.array or float list (optional)
        Data points of the trajectory's velocity to encode.
    time : np.array or float list or float
        If the provided time is an array of floats with the same length as the
        `pos_data` parameter, it will be treated as the time samlpes of the trajecoty.
        In case this parameter is a single float, it will be treated as the sample
        time of the trajectory and the time vector will be computed accordingly.
    num_weights : int, optional
        The desired number of weights to estimate the DMP.
    a_z : float, optional
        The a_z gain of the DMP.
    a_x : float, optional
        the a_x gain of the DMP.

    """
    a_x = None
    a_z = None
    b_z = None
    num_weights = None
    y0 = None
    goal = None
    _num_dof = None
    _d_t = None

    def __init__(self, pos_data=None, time=None, vel_data=[], num_weights=25, a_z=48.0, a_x=2.0, r = 1, goal= None, amp = 1):

        if pos_data is not None:
            # Copy the parameters into the appropriate class attributes
            self.a_x = a_x
            self.a_z = a_z
            #self.b_z = self.a_z / 4
            self.b_z = np.sqrt(self.a_z)/4
            self.num_weights = num_weights
            self._pos_training_data = np.asarray(pos_data.copy())
            self._vel_training_data = np.asarray(vel_data.copy())
            self.y0 = self._pos_training_data[0, :]
            # We assume that the number of trajectory samples are smaller then the DOF
            self._num_samples = np.max(self._pos_training_data.shape)
            self._num_dof = np.min(self._pos_training_data.shape)
            
            self.goal = goal
            if self.goal is None:
                minn = np.min(self._pos_training_data)
                maxx = np.max(self._pos_training_data)
                self.goal = 0.5*(minn+maxx)
                self.goal = np.reshape(self.goal, (1, self._pos_training_data.shape[1]))            
        
            try:
                if not(len(np.asarray(time).shape)):
                    # If the provided argument is a number we treat is as sample time
                    self._time_vec = np.arange(0, self._num_samples*time, time)
                    self._d_t = time
                else:
                    # If the provided argument an array we treat it as an array
                    # of time stamps
                    if len(time) != self._num_samples:
                        raise Exception("Time stamp vector length does not match the "
                                        "number of samples !!\n"
                                        ">> Num. samples: {0} | Len. time: {1} <<"
                                        .format(self._num_samples, len(time)))
                    else:
                        self._time_vec = np.asarray(time) - time[0]
                        self._d_t = np.mean(np.diff(self._time_vec))
            except Exception as e:
                print('Exception when dealing with the "time" argument:\n{0}'.format(e))
                return
            
            if self._time_vec[0] == 0:
                self._time_vec += 0.5*self._d_t
            
             # Tau equals to the duration of the trajectory
            self.tau = self._time_vec[-1]
            self.amp = amp
            
            # Initial conditions 
            self.d_phi = 2*np.pi / self.tau # Angular velocity
            
            print("d_phi= ", self.d_phi)
            self.initial_phi = 0
            #self.initial_r = 1
            self.r = r

            # Prepare the Gaussian kernel functions
            self._prepare_gaussian_kernels()

            # Encode the DMP
            self.__encode_dmp()

    def _prepare_gaussian_kernels(self):
        
        #self.c = np.exp(-self.a_x * np.linspace(0, 1, self.num_weights))
        #self.sigma = np.square((np.diff(self.c)*0.75))
        #self.sigma = np.append(self.sigma, self.sigma[-1])
        
        #self.c = np.exp(-self.a_x * np.linspace(0, 2*np.pi, self.num_weights))
        self.c = np.linspace(0,2*np.pi, self.num_weights)
        
    def __encode_dmp(self):
        y = self._pos_training_data

        dy = np.zeros(y.shape)
        
        # For periodic DMP, velocity is constant
        #dy[:,0 ] = self.d_phi/self.r
        
        if not(len(self._vel_training_data)):
            # Estimate the velocities if needed
            for i in range(self._num_dof):
                dy[:, i] = np.divide(np.gradient(y[:, i]), np.gradient(self._time_vec))
        else:
            dy = self._vel_training_data

        # Estimate the accelerations
        ddy = np.zeros(dy.shape)
        for i in range(self._num_dof):
            ddy[:, i] = np.divide(np.gradient(dy[:, i]), np.gradient(self._time_vec))

        # Prepare empty matrices
        ft = np.zeros((self._num_samples, self._num_dof), dtype=np.float32)
        A = np.zeros((self._num_samples, self.num_weights), dtype=np.float32)

        # Define the phase vector
        #x = np.exp(-self.a_x * self._time_vec / self.tau)
        #x = np.exp(np.linspace(0,2*np.pi, len(self._time_vec)))
        x = np.linspace(0,2*np.pi, len(self._time_vec))
        
        # Estimate the forcing term
        for dof in range(self._num_dof):
            #ft[:, dof] = ddy[:, dof]*np.square(self.tau) - \
                #self.a_z * (self.b_z * (y[-1][dof] - y[:, dof]) - dy[:, dof] * self.tau)
            ft[:, dof] = (ddy[:, dof]/(self.d_phi**2)) - \
                self.a_z * (self.b_z * (y[-1][dof] - y[:, dof]) - (dy[:, dof]/ self.d_phi))

        for i in range(self._num_samples):
            #Normal DMP
            #psi = np.exp(np.divide(-0.5 * np.square(x[i] - self.c), self.sigma))
            
            #Periodic DMP
            psi = np.exp( np.cos(x[i] - self.c) -1 )
            
            A[i, :] =  np.divide(psi, np.sum(psi)) #/ self.r#* x[i]  

        # Do linear regression in the least square sense
        self.weights_pos = np.transpose(np.linalg.lstsq(A, ft)[0])

    def __decode_dmp(self):
        pass

    def _integrate_step(self, x, y, z):
        # Phase variable
        # dx = (-a_x * x) / tau
        # x = x + dx * dt
        #dx = -self.a_x * x / self.tau
        
        # dx = d(Phi)
        # tau * dPhi = 1
        # so dPhi = 1/tau
        #print(self.d_phi)
        dx = self.d_phi
        
        x = x + dx * self._d_t

        # The weighted sum of the locally weighted regression models
        # psi = exp(-(x - c)^2 / (2 * sigma))
        
        # Normal DMP
        #psi = np.exp(- np.square(x-self.c) /
        #             (np.multiply(self.sigma, 2)))
        
        # Periodic DMP
        psi = np.exp(np.cos(x - self.c) -1 )
        
        for dof in range(self._num_dof):
            # Forcing function
            # sum( (w(dof) * x) * psi/sum(psi) )
            
            #fx = sum(np.multiply(
            #    (np.multiply(self.weights_pos[dof], x)),
            #    (np.divide(psi, sum(psi)))
            #))
            
            fx = sum(np.multiply(
                (np.multiply(self.weights_pos[dof], self.r)),
                (np.divide(psi, sum(psi)))
            ))

            # Derivatives
            # dz = a_z * (a_z/4 * (goal - y) - z) + fx
            
            #dz = self.a_z * \
            #    (self.b_z *
            #        (self.goal[dof] - y[dof])
            #        - z[dof]) + fx

            # For periodic DMP            
            dz = self.d_phi * (self.a_z *(self.b_z*(self.goal[dof]-y[dof]) - z[dof]) + fx )
            
            dy = z[dof] * self.d_phi

            # Temporal scaling
            #dz = dz / self.tau
            #dy = dy / self.tau

            # Integration
            z[dof] = z[dof] + dz * self._d_t
            y[dof] = y[dof] + dy * self._d_t

        return x, y, z

    def decode(self):
        """Function decodes the DMP and returns a trajectory.

        Returns
        ----------
        traj : np.array
            The decoded trajectory
        t : np.array
            The time samples for the decoded trajectory
        """
        # Initial states
        y = np.asarray(self.y0)
        z = np.zeros(self._num_dof)
        
        x = self.initial_phi

        # Set a limit for the phase
        self.x_max = 2*np.pi # np.exp(-self.a_x)

        # First sample equals y0
        #traj = [y]
        traj = [y.copy()]
        vel = []
        #t = [0]
        t = [0]
        # Decode loop
        while x < self.x_max:
            #print(x,y,z)
            
            #if x > np.pi:
            #    self.goal = np.array([0.5])
            [x, y, z] = self._integrate_step(x, y, z)
            #print(tuple(y))
            
            #self.amp = self.amp + 0.005
            #y2 = y * self.amp
            #self.r = self.r*1.001
            
            traj.append(tuple(y))
            #traj.append(tuple(y2))
            vel.append(tuple(z*self.d_phi))
            t.append(t[-1]+self._d_t)

        traj = np.asarray(traj)
        traj = traj#[1:] # Idk why, but without this, the 0th element is assigned the same value as the last one. wtf.
        
        vel = np.asarray(vel)
        return traj, vel, t

    def step_decode(self):
        pass

def main():
    """ Put this in a separate function, so the main() can be run even if this module is imported"""
    # Test the Periodic DMP

    T= 10
    D_T = 0.01

    initial_angle = 0

    num_weights = 25
    a_z = 48

    r  = 1
    # amp = 1.1

    time_vec = np.arange(0, T, D_T)
    input_traj = np.array([-np.cos(8*np.pi*time_vec/time_vec[-1]),  ]).transpose()
    input_vel = None

    #traj = np.array([np.sin((8*np.pi)*time_vec/time_vec[-1]),]).transpose()

    #goal = np.array([traj[-1]])
    goal = np.array([-1])

    encoded_dmp = PeriodicDMP(input_traj, time_vec, num_weights = num_weights, a_z = a_z, r = r, goal = goal)

    # Decode the DMP
    decoded_traj, decoded_vel, _ = encoded_dmp.decode()
      
    # Move the goal down a bit,
    #goal = goal + 1
    
    # Increase the r a bit
    # r = r * 1.1

    # Check the output
    # For first period, boundary velocity at the beginning must be zero. boundary velocity at end MUST NOT be zero.
   
    return input_traj, input_vel, decoded_traj, decoded_vel

if __name__ == '__main__':
    main()
    
    
