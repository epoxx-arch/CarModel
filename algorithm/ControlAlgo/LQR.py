
import random
import numpy as np 

"""
Vehicle Model:
"""
# Add lambda functions
cos = lambda a : np.cos(a)
sin = lambda a : np.sin(a)
tan = lambda a : np.tan(a)

args = {'wheelbase':2.94, 'steer_angle_limits': [-1.0, 1.0], 'acc_limits': [-5.5, 2.0],
         'max_speed':30.0, 'timestep': 0.1, 'horizon': 40, 'num_states': 4, 'num_ctrls': 2}

class Model:
    """
    A vehicle model with 4 dof. 
    State - [x, y, vel, theta]
    Control - [acc, yaw_rate]
    """
    def __init__(self, args):
        self.wheelbase = args['wheelbase']
        self.steer_min = args['steer_angle_limits'][0]
        self.steer_max = args['steer_angle_limits'][1]
        self.accel_min = args['acc_limits'][0]
        self.accel_max = args['acc_limits'][1]
        self.max_speed = args['max_speed']
        self.Ts = args['timestep']
        self.N = args['horizon']
        self.z = np.zeros((self.N))
        self.o = np.ones((self.N))
        self.l  = 2.94
        
    def forward_simulate(self, state, control):
        """
        Find the next state of the vehicle given the current state and control input
        """
        # Clips the controller values between min and max accel and steer values
        control[0] = np.clip(control[0], self.accel_min, self.accel_max)
        control[1] = np.clip(control[1], state[2]*tan(self.steer_min)/self.wheelbase, state[2]*tan(self.steer_max)/self.wheelbase)
        
        next_state = np.array([state[0] + cos(state[3])*(state[2]*self.Ts + (control[0]*self.Ts**2)/2),
                               state[1] + sin(state[3])*(state[2]*self.Ts + (control[0]*self.Ts**2)/2),
                               np.clip(state[2] + control[0]*self.Ts, 0.0, self.max_speed),
                               state[3] + control[1]*self.Ts])  # wrap angles between 0 and 2*pi - Gave me error
        return next_state  

    def get_A_matrix(self, velocity_vals, theta, acceleration_vals):
        """
        Returns the linearized 'A' matrix of the ego vehicle 
        model for all states in backward pass. 
        """
        v = velocity_vals
        v_dot = acceleration_vals
        A = np.array([[self.o, self.z, cos(theta)*self.Ts, -(v*self.Ts + (v_dot*self.Ts**2)/2)*sin(theta)],
                      [self.z, self.o, sin(theta)*self.Ts,  (v*self.Ts + (v_dot*self.Ts**2)/2)*cos(theta)],
                      [self.z, self.z,             self.o,                                         self.z],
                      [self.z, self.z,             self.z,                                         self.o]])
        return A

    def get_B_matrix(self, theta):
        """
        Returns the linearized 'B' matrix of the ego vehicle 
        model for all states in backward pass. 
        """
        B = np.array([[self.Ts**2*cos(theta)/2,         self.z],
                      [self.Ts**2*sin(theta)/2,         self.z],
                      [         self.Ts*self.o,         self.z],
                      [                 self.z, self.Ts*self.o]])
        return B
    
    def LQRA(self,V,phi):
        A = np.array([[0,0,-V*np.sin(phi)],[0,0,V*np.cos(phi)],[0,0,0]]) 
        return A 
    
    def LQRB(self,V,phi):
        B = np.array([[np.cos(phi),0],[np.sin(phi),0],[np.tan(phi) / self.l, V / (self.l * np.cos(phi) ** 2)]])
        return B 

"""
LQR 算法 实现：

METHOD ONE: Ricate Recursion
"""

class LQR():
    def __init__(self,A,B,Q,R) -> None:
        self.epision = 1e-6
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.P = np.eye(4)
        self.K = np.zeros((2,4))


    def get_K(self):
        return self.R + (self.B.T @ self.P @ self.B).I @ self.B.T @ self.P @ self.A
    
    def get_P(self):
        while True:
            P_next = self.Q + self.A.T @ self.P @ self.A - self.A.T @ self.P @ self.B @ (self.R + self.B.T @ self.P @ self.B).I @ self.B.T @ self.P @ self.A
            if np.linalg.norm(P_next - self.P) < self.epision:
                break
            self.P = P_next

    def get_control(self, state):
        self.P = np.eye(4)
        self.get_P()
        self.K = self.get_K()
        return -self.K @ state

def generate_circle_traj_point():
    """
    Generate a circle trajectory for the ego vehicle to follow
    """
    radius = 10.0
    center = np.array([0, 0])
    theta = np.linspace(0, 2*np.pi, 100)
    x = center[0] + radius*np.cos(theta)
    y = center[1] + radius*np.sin(theta)
    a = [(random.random()) - 1.0 for _ in range(len(x))]
    return x, y,a



if __name__ == "__main__":
    state = np.array([0, 0, 0])

    x,y,a = generate_circle_traj_point()
    model = Model(args)
    plot_state = []
    plot_track_s = []
    for _x,_y,_a in zip(x,y,a):
        state = np.array([_x, _y,_a,0.0])
        plot_state.append([x,y])
        lqr = LQR(state)
        control = lqr.get_control(state)
        new_x,new_y = model.forward_simulate(state, control)
        plot_track_s.append([new_x,new_y])
    
    import matplotlib.pyplot as plt
    plt.plot([i[0] for i in plot_state], [i[1] for i in plot_state])
    plt.plot([i[0] for i in  plot_track_s], [i[1] for i in plot_track_s])
    plt.show()