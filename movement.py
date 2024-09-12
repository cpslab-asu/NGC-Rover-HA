import numpy as np
import matplotlib.pyplot as plt

class Movement:
    def __init__(self):
        self.dt_integral = 0.01
        self.delta_t = 1
        self.history_t = [0.0]
        self.history_x = [0.0]
        self.history_y = [0.0]
        self.history_theta = [0.0]
        self.targetAngle = 0
        self.baseAngle = 0
        self.err_threshold = 1e-3
        
    def __call__(self, v, omega, targetAngle):
        x = self.history_x[-1]
        y = self.history_y[-1]
        theta = self.history_theta[-1]

        for t in np.arange(0, self.delta_t, self.dt_integral):
            if abs(theta-targetAngle) < self.err_threshold:
                omega = 0.0
            y += v * np.sin(theta) * self.dt_integral
            x += v * np.cos(theta) * self.dt_integral
            theta += omega * self.dt_integral
            
            # Store the positions
            self.history_x.append(x)
            self.history_y.append(y)
            self.history_theta.append(theta)
            self.history_t.append(self.history_t[-1]+self.dt_integral)
    
    def getCompass(self):
        return self.history_theta[-1]
    
    def getGPS(self):
        return (self.history_x[-1], self.history_y[-1])
    
# Plotting the trajectory
# plt.figure(figsize=(8, 8))

rover = Movement()

rover(0,0, 0)

# rover(2,0, 0)
# baseAngle = rover.getCompass()
# targetAngle = baseAngle + np.pi/2
# rover(1,np.pi/4, targetAngle)
# baseAngle = rover.getCompass()
# targetAngle = baseAngle + np.pi/2
# rover(1,np.pi/4, targetAngle)
# rover(1,np.pi/4, targetAngle)
# rover(1,np.pi/4, targetAngle)
# baseAngle = rover.getCompass()
# targetAngle = baseAngle + np.pi/2
# rover(1,np.pi/4, targetAngle)
# rover(1,np.pi/4, targetAngle)
# rover(1,np.pi/4, targetAngle)

# plt.figure(figsize=(8, 8))
# plt.plot(rover.history_x, rover.history_y, "-")
# plt.title('Trajectory of the Point')
# plt.xlabel('X position (m)')
# plt.ylabel('Y position (m)')
# plt.legend()
# plt.grid(True)
# plt.axis('equal')
# plt.savefig("test.png")