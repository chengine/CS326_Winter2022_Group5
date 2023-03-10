import numpy as np
import scipy
import scipy.interpolate
from scipy.integrate import cumtrapz

class State:
    def __init__(self,x,y,V,th):
        self.x = x
        self.y = y
        self.V = V
        self.th = th

    @property
    def xd(self):
        return self.V*np.cos(self.th)

    @property
    def yd(self):
        return self.V*np.sin(self.th)

def compute_controls(traj):
    """
    Input:
        traj (np.array shape [N,7])
    Outputs:
        V (np.array shape [N]) V at each point of traj
        om (np.array shape [N]) om at each point of traj
    """

    V = np.sqrt(traj[:, 3]**2+traj[:, 4]**2)
    om = (traj[:, 3]*traj[:, 6]-traj[:, 4]*traj[:, 5])/(traj[:, 3]**2+traj[:, 4]**2)

    return V, om

def compute_arc_length(V, t):
    """
    This function computes arc-length s as a function of t.
    Inputs:
        V: a vector of velocities of length T
        t: a vector of time of length T
    Output:
        s: the arc-length as a function of time. s[i] is the arc-length at time
            t[i]. This has length T.
    """

    s = cumtrapz(V, t, initial=0)

    return s

def rescale(s, V, om, V_max, a_max, om_max):
    """
    This function computes V_tilde, given the unconstrained solution V, and om.
    Inputs:
        V: vector of velocities of length T. Solution from the unconstrained,
            differential flatness problem.
        om: vector of angular velocities of length T. Solution from the
            unconstrained, differential flatness problem.
        V_max: maximum velocity
        a_max: maximum acceleration
        om_max: maximum angular velocity
    Output:
        V_tilde: Rescaled velocity that satisfies the control constraints.
        om_tilde: Rescaled angular velocity that satisfies the constrol constraints.
        tau: Rescaled timestamps computed from V_tilde.
    """

    # velocity to satisfy both V_max and om_max
    V_tilde = np.sign(V)/np.max(np.abs([np.ones(np.shape(V))/V_max, 1/V*om/om_max]), axis=0)

    # forwards pass
    V_tilde[0] = 0
    for i in range(len(V_tilde)-1):
        Vi = V_tilde[i]
        ds = s[i+1]-s[i]
        Vf_max = np.sqrt(Vi**2 + 2*a_max*ds)
        V_tilde[i+1] = min(Vf_max, V_tilde[i+1])
    
    # backwards pass
    V_tilde[-1] = 0
    for i in range(len(V_tilde)-1, 0, -1):
        Vf = V_tilde[i]
        ds = s[i]-s[i-1]
        Vi_max = np.sqrt(Vf**2 + 2*a_max*ds)
        V_tilde[i-1] = min(Vi_max, V_tilde[i-1])

    # compute angular velocity
    om_tilde = om*V_tilde/V

    # compute timestamps
    tau = np.zeros_like(V_tilde)
    for i in range(1, len(tau)):
        ds = s[i]-s[i-1]
        Vi, Vf = V_tilde[i-1], V_tilde[i]
        accel = (Vf**2 - Vi**2)/(2*ds)
        if abs(accel) > 1e-6:
            tau[i] = tau[i-1] + (Vf-Vi)/accel
        elif abs(Vi) > 1e-6:
            tau[i] = tau[i-1] + ds/Vi
        else:
            raise ValueError('zero velocity at i = ' + str(i))

    return V_tilde, om_tilde, tau

def interpolate_traj(traj, tau, V_tilde, om_tilde, dt, s_f):
    """
    Inputs:
        traj (np.array [N,7]) original unscaled trajectory
        tau (np.array [N]) rescaled time at orignal traj points
        V_tilde (np.array [N]) new velocities to use
        om_tilde (np.array [N]) new rotational velocities to use
        dt (float) timestep for interpolation

    Outputs:
        t_new (np.array [N_new]) new timepoints spaced dt apart
        V_scaled (np.array [N_new])
        om_scaled (np.array [N_new])
        traj_scaled (np.array [N_new, 7]) new rescaled traj at these timepoints
    """
    
    # Get new final time
    tf_new = tau[-1]

    # Generate new uniform time grid
    N_new = int(tf_new/dt)
    t_new = dt*np.array(range(N_new+1))

    # Interpolate for state trajectory
    traj_scaled = np.zeros((N_new+1,7))
    traj_scaled[:,0] = np.interp(t_new,tau,traj[:,0])   # x
    traj_scaled[:,1] = np.interp(t_new,tau,traj[:,1])   # y
    traj_scaled[:,2] = np.interp(t_new,tau,traj[:,2])   # th
    # Interpolate for scaled velocities
    V_scaled = np.interp(t_new, tau, V_tilde)           # V
    om_scaled = np.interp(t_new, tau, om_tilde)         # om
    # Compute xy velocities
    traj_scaled[:,3] = V_scaled*np.cos(traj_scaled[:,2])    # xd
    traj_scaled[:,4] = V_scaled*np.sin(traj_scaled[:,2])    # yd
    # Compute xy acclerations
    traj_scaled[:,5] = np.append(np.diff(traj_scaled[:,3])/dt,-s_f.V*om_scaled[-1]*np.sin(s_f.th)) # xdd
    traj_scaled[:,6] = np.append(np.diff(traj_scaled[:,4])/dt, s_f.V*om_scaled[-1]*np.cos(s_f.th)) # ydd

    return t_new, V_scaled, om_scaled, traj_scaled


def compute_smoothed_traj(path, V_des, alpha, dt):
    """
    Fit cubic spline to a path and generate a resulting trajectory for our
    wheeled robot.

    Inputs:
        path (np.array [N,2]): Initial path
        V_des (float): Desired nominal velocity, used as a heuristic to assign nominal
            times to points in the initial path
        alpha (float): Smoothing parameter (see documentation for
            scipy.interpolate.splrep)
        dt (float): Timestep used in final smooth trajectory
    Outputs:
        traj_smoothed (np.array [N,7]): Smoothed trajectory
        t_smoothed (np.array [N]): Associated trajectory times
    """
    
    path = np.array(path)

    t_nominal = np.zeros(path.shape[0])
    t_nominal[1:] = np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=-1)/V_des)

    tck, _ = scipy.interpolate.splprep([path[:,0], path[:,1]], u=t_nominal, s=alpha)

    t_smoothed = np.linspace(0, t_nominal[-1], int(t_nominal[-1]/dt))

    x_d, y_d = scipy.interpolate.splev(t_smoothed, tck)
    xd_d, yd_d = scipy.interpolate.splev(t_smoothed, tck, der=1)
    xdd_d, ydd_d = scipy.interpolate.splev(t_smoothed, tck, der=2)

    theta_d = np.zeros(t_smoothed.shape)
    theta_d[:-1] = np.arctan2(np.diff(y_d), np.diff(x_d))
    theta_d[-1] = theta_d[-2]

    traj_smoothed = np.stack([x_d, y_d, theta_d, xd_d, yd_d, xdd_d, ydd_d]).transpose()

    return traj_smoothed, t_smoothed

def modify_traj_with_limits(traj, t, V_max, a_max, om_max, dt):
    """
    Modifies an existing trajectory to satisfy control limits and
    interpolates for desired timestep.

    Inputs:
        traj (np.array [N,7]): original trajecotry
        t (np.array [N]): original trajectory times
        V_max, om_max (float): control limits
        dt (float): desired timestep
    Outputs:
        t_new (np.array [N_new]) new timepoints spaced dt apart
        V_scaled (np.array [N_new])
        om_scaled (np.array [N_new])
        traj_scaled (np.array [N_new, 7]) new rescaled traj at these timepoints
    """

    V,om = compute_controls(traj=traj)
    s = compute_arc_length(V, t)
    V_tilde, om_tilde, tau = rescale(s, V, om, V_max, a_max, om_max)

    t_new, V_scaled, om_scaled, traj_scaled = tau, V_tilde, om_tilde, traj
    
    return t_new, V_scaled, om_scaled, traj_scaled