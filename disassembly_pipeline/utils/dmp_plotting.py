import time
import numpy as np
import matplotlib.pyplot as plt

class FT_plotter:
    def __init__(self, robot_obj, dt = 0.02, max_t_diff = 1, fn_F = 'graphs/F.jpg', fn_M = 'graphs/M.jpg',
                 fn_ang='graphs/ang.jpg', dpi = 300, expected_t_of_msr = 100):
        self.dt = dt # In seconds
        self.robot_obj = robot_obj
        self.fn_F = fn_F
        self.fn_M = fn_M
        self.fn_ang = fn_ang
        self.dpi = dpi
                
        self.n = 0
        #expected_t_of_msr = 30 # How long to make array :
        n_arr = int(expected_t_of_msr / self.dt)
        self.l = np.empty((n_arr, 6))
        
        self.max_t_diff = max_t_diff # 
        
        self.last_t = time.time()
        
        self.ang = []
        self.cond = []
        
    def execute(self, d_ang = None, cond = None):
        #print(self.n)
        t = time.time()
        if (t - self.last_t)> self.dt:
            #print("HIT")
            self.last_t = t
            FT = self.robot_obj.last_franka_state.K_F_ext_hat_K
            
            self.l[self.n, :] = FT
            
            if d_ang is not None:self.ang.append(d_ang)
            if cond is not None:self.cond.append(cond)
            
            self.n +=1
            
    def on_exit(self):
        # Save
        out = self.l
        
        # plot fig 1
        plt.clf()
        plt.grid()
        last_t = self.n/self.dt
        tt = np.linspace(0,last_t, len(out[:self.n,0]))
        
        plt.plot(tt, out[:self.n,0],'r', label='F_x')
        plt.plot(tt, out[:self.n,1],'g', label = 'F_y')
        plt.plot(tt, out[:self.n,2], 'b', label='F_z')
        plt.xlabel("t [s]")
        plt.legend()
        plt.savefig(self.fn_F, dpi=self.dpi)
         
        plt.clf()
        # Plot fig 2 
        plt.grid()
        plt.plot(tt, out[:self.n,3], 'r', label='M_x')
        plt.plot(tt, out[:self.n,4], 'g', label='M_y')
        plt.plot(tt, out[:self.n,5], 'b', label = 'M_z')
        plt.plot(tt, self.cond, 'k', label='cond')
        plt.xlabel("t [s]")

        plt.legend()
        plt.savefig(self.fn_M, dpi = self.dpi)
        
        plt.clf()
        plt.grid()
        plt.plot(tt, self.ang, 'r', label='d_angle')
        plt.xlabel("t [s]")
        plt.legend()
        plt.savefig(self.fn_ang, dpi = self.dpi)
        
        
        np.save(file = 'graphs/M.npy', arr = self.l[:self.n, 3])
        
    def condition(self):
        max_d = int(self.max_t_diff / self.dt)
        #max_d = 2#30
        d = np.min((self.n, max_d))
        if self.n<2: return 0
        v = np.max((self.l[self.n-d : self.n-1, 3]) - np.min(self.l[self.n-d:self.n-1, 3]))
        
        #v = np.max((self.l[self.n-d : self.n, 3])) - self.l[self.n-1, 3]
        
        
        #v = np.max((self.l[self.n-d : self.n, 3]) - np.min(self.l[self.n-d:self.n, 3]))
        return v 
