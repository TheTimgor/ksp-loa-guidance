import numpy as np
from numpy import exp, sqrt, cos
import matplotlib.pyplot as plt

# fun(p, p`)
# t is values to solve over, linspace is ideal here
def solve_o2(fun, t_vals, y_con, dy_con):
	y = y_con  # start at starting condition
	dy = dy_con  # derivative

	y_predicted = [y]
	dy_predicted = [dy]

	for t0, t1 in zip(t_vals[:-1],t_vals[1:]):

		d2y = fun(y, dy)  # second derivative
		
		# values for our quadratic approximation
		a = d2y/2
		b = dy - d2y*t0
		c = y - a*t0**2 - b*t0

		y = a*t1**2 + b*t1 + c
		dy = 2*a*t1 + b

		y_predicted.append(y) 
		dy_predicted.append(dy)

	return y_predicted, dy_predicted

t = np.linspace(-5,5,100)

def accel(p, v):
	a = np.array([0,0,-2]) - 0.3*v 
	return a
	

pvals, _ = solve_o2(accel, t, np.array([0,0,0]), np.array([10,10,15]))
pvals = np.array(pvals)
print(pvals)

x = pvals[:,0]
y = pvals[:,1]
z = pvals[:,2]

ax = plt.axes(projection='3d')
ax.plot(x,y,z)
plt.show()

# t = np.linspace(0,10,100)
# plt.plot(t, exp(-t/2)*cos(( sqrt(3)/2)*t ), "b", label='actual')
# o, _ = solve_o2(lambda y,dy: -y - dy, t, 1, -0.5)
# print(len(o))
# plt.plot(t, o, "r", label='predicted')
# plt.legend()
# plt.show()