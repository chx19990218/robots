#/usr/bin/python3.6
from casadi import *
import numpy as np
# define symbolic variables for modeling
x = SX.sym("x")
y = SX.sym("y")
z = SX.sym("z")
phi = SX.sym("phi")
theta = SX.sym("theta")
psi = SX.sym("psi")
xdot = SX.sym("xdot")
ydot = SX.sym("ydot")
zdot = SX.sym("zdot")
phidot = SX.sym("phidot")
thetadot = SX.sym("thetadot")
psidot = SX.sym("psidot")
state = vertcat(x, y, z,
				phi, theta, psi,
				xdot, ydot, zdot,
				phidot, thetadot, psidot)
length_state = state.shape[0]
u1 = SX.sym("u1")
u2 = SX.sym("u2")
u3 = SX.sym("u3")
u4 = SX.sym("u4")
control = vertcat(u1, u2, u3, u4)
#control 
length_control = control.shape[0]
length_state_control = length_control + length_state

# define dynamics parameters
Ixx = 1.2
Iyy = 1.2
Izz = 2.3
k = 0.2225
l = 0.26
m = 0.52
b = 0.002
g = 9.81

# euler angles
Rz = np.array([[cos(psi), -sin(psi), 0],
			[sin(psi), cos(psi), 0],
			[0, 0, 1]])
Ry = np.array([[cos(theta), 0, sin(theta)],
			[0, 1, 0],
			[-sin(theta), 0, cos(theta)]])
Rx = np.array([[1, 0, 0],
			[0, cos(phi), -sin(phi)],
			[0, sin(phi), cos(phi)]])
EulerAngleType = "ZYX"
if EulerAngleType == "XYZ":
	R = Rx @ Ry @ Rz
	W = np.array([[cos(theta)*cos(psi), sin(psi), 0],
				[-cos(theta)*cos(psi), sin(psi), 0],
				[sin(theta), 0, 1]])
elif EulerAngleType == "ZYX":
	R = Rz @ Ry @ Rx
	W = np.array([[1, 0, -sin(theta)],
				[0, cos(phi), cos(theta)*sin(phi)],
				[0, -sin(phi), cos(theta)*cos(phi)]])
				
# Jacobian(relate body frame angular velocities to the rates of euler angles)
I = np.array([[Ixx, 0, 0],
			[0, Iyy, 0],
			[0, 0, Izz]])
J = W.T @ I @ W
eulerangle = np.array([phi, theta, psi]).T
eulerangledot = np.array([phidot, thetadot, psidot]).T

# Coriolis forces
dJ_deulerangle_flat = jacobian(J, eulerangle)
dJ_dt_flat = dJ_deulerangle_flat @ eulerangledot
dJ_dt = reshape(dJ_dt_flat, 3, 3)

etadot_J = [phidot, thetadot, psidot] @ J
grad_etadot_J = jacobian(etadot_J, eulerangle)
C = dJ_dt - 1/2*grad_etadot_J.T

# Torques in the direction of phi, theta, psi
tau_beta = np.array([[l*k*(-u2 + u4)], [l*k*(-u1 + u3)], [b*(-u1+u2-u3+u4)]])
# total thrust
Thrust = k*(u1 + u2 + u3 + u4)

# Dynamics
rhs = SX.sym("rhs", 12, 1)
rhs[0] = xdot
rhs[1] = ydot
rhs[2] = zdot
rhs[3] = phidot
rhs[4] = thetadot
rhs[5] = psidot

# Equations for COM configuration
rhs[6:9] = -g*np.array([[0],[0],[1]]) + R @ np.array([[0],[0],[Thrust]])/m
# Euler Lagrange equations for angular dynamics
rhs[9:12] = solve(J, tau_beta - C @ eulerangledot)


# build function object
f = Function('f', [state, control], [rhs]);

# sampling period
T = 0.1;
# prediction horizon
N = 10;
# decision variables
U = SX.sym('U', length_control, N);
# parameters(init state + reference states)
P = SX.sym('P', length_state, N+1);
# state variables(current state + predicted states)
X = SX.sym('X', length_state, N+1);
# object function
obj = 0;
# constraints
G = SX.zeros(1,1);
# state weighting matrices
Q = np.zeros((12,12));
Q[0:3,0:3] = 1*np.eye(3);
Q[6:9,6:9] = 0.2*np.eye(3);
# control weighting matrices
R = 0.001*np.eye(4);
# init state
st = X[:,0];
# init state constraint
G = vertcat(G, st-P[:,0]);
for k in range(N):
	st = X[:,k]
	st_next = X[:,k+1]
	con = U[:,k]
	obj = obj + (st - P[:,k+1]).T @ Q @ (st - P[:,k+1]) + U[:,k].T @ R @ U[:,k]
	f_value = f(st, con)
	st_next_euler = st + T * f_value
	G = vertcat(G,st_next - st_next_euler)
OPT_variables = vertcat(reshape(X,length_state*(N+1),1), reshape(U,length_control*N,1))
#nlp_prob ={'f':obj, 'x':OPT_variables, 'g':G, 'p':P}
nlp_prob ={'f':obj, 'x':OPT_variables, 'p':P}
opts = {'print_time':0}
solver = nlpsol('solver', 'ipopt', nlp_prob)


p1 = SX.zeros(12, 11)

sol = solver(x0 = SX.zeros(1,1), 
						 lbg = SX.zeros(length_state*(N+1), 1), 
						 ubg = SX.zeros(length_state*(N+1), 1), 
						 lbx = SX.zeros(length_state*(N+1)+length_control*N, 1), 
						 ubx = SX.zeros(length_state*(N+1)+length_control*N, 1), 
						 p = SX.zeros(length_state*(N+1)+length_control*N, 1));
print(solver)

