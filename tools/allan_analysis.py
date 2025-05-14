import numpy as np
import allantools
import matplotlib.pyplot as plt

# 1) load your CSV: columns [timestamp, ax, ay, az]
data = np.loadtxt('static_accel.csv', delimiter=',', skiprows=1)
ts = data[:,0]              # in seconds
ax = data[:,1] * 9.80665    # convert g→m/s² if needed

# 2) resample to uniform rate (if ts is irregular)
fs = 100.0  # sample rate in Hz
N = int((ts[-1]-ts[0])*fs)
t_uniform = np.linspace(ts[0], ts[-1], N)
ax_u = np.interp(t_uniform, ts, ax)

# 3) compute Allan deviation
taus, adev, _, _ = allantools.oadev(ax_u, rate=fs, data_type='freq')  # use data_type='dev' for deviation
# note: for acceleration, pass data_type='dev' instead of 'freq'

# 4) plot
plt.loglog(taus, adev)
plt.xlabel('Cluster time τ [s]')
plt.ylabel('ADEV [m/s²]')
plt.title('Allan Deviation – Static Accel X')
plt.grid(True, which='both')
plt.show()