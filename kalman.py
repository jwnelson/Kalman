import numpy as np 
import matplotlib.pyplot as plt 

#setup initial parameters
n_iter = 50
sz = (n_iter,)	#Size of array
x0 = -0.5	#Initial position value
v = 0.05	#Velocity
R = 0.1 # Estimate of measurement variance

Q = 1e-6 #Process variance

#Create the arrays
xhat = np.zeros(sz)	# a posteri estimate of x
P = np.zeros(sz)	# a posteri error estimate
xhatminus = np.zeros(sz)	# a priori estimate of x
Pminus = np.zeros(sz)	# a priori error estimate
K = np.zeros(sz)	# gain or blending factor
z = np.zeros(sz)	# measurement array
x = np.zeros(sz)	# array of position values



# initial guess
xhat[0] = 0.0
P[0] = 1.0

x[0] = x0

for k in range(1, n_iter):
	# move the robot
	x[k] = x[k-1] + v

	# generate a measurement about x
	z[k] = np.random.randn()*R + x[k]
	# time update
	xhatminus[k] = xhat[k-1]
	Pminus[k] = P[k-1] + Q

	# measurement update
	K[k] = Pminus[k]/( Pminus[k]+R )
	xhat[k] = xhatminus[k] + K[k]*(z[k]-xhatminus[k])
	P[k] = (1-K[k])*Pminus[k]

plt.figure()
plt.plot(z, 'k+', label='noisy measurements')
plt.plot(xhat, 'b-', label='a posteri estimate')
plt.plot(x, '^', label='actual position')
#plt.axhline(x, color='g', label = 'truth value')
plt.legend()
plt.title('Estimate vs. iteration step', fontweight='bold')
plt.xlabel('Iteration')
plt.ylabel('Voltage')

plt.figure()
valid_iter = range(1,n_iter)
plt.plot(valid_iter, Pminus[valid_iter], label = 'a priori error estimate')
plt.title('Estimated $\it\mathbf{a \ priori}}$ error vs. iteration step', fontweight='bold')
plt.xlabel('Iteration')
plt.ylabel('$(Voltage)^2$')
plt.setp(plt.gca(), 'ylim', [0, 0.01])
plt.show()