import numpy as np
import matplotlib.pyplot as plt

# Load the data from CSV files
x = np.loadtxt('x.csv', delimiter=',')
y = np.loadtxt('y.csv', delimiter=',')
a = np.loadtxt('a.csv', delimiter=',')
b = np.loadtxt('b.csv', delimiter=',')

# Calculate differences
nx = a - x
ny = b - y

# Plot the data
#plt.figure()
#plt.plot(x, y, 'xb')
#plt.plot(a, b, '+r')
#plt.title('noisy co-ordinates vs real co-ordinates')

# Track the objects using Kalman filter
def kalmanTracking(z):
    dt = 0.2  # time interval
    N = len(z)  # number of samples

    F = np.array([[1, dt, 0, 0], [0, 1, 0, 0], [0, 0, 1, dt], [0, 0, 0, 1]])  # CV motion model
    Q = np.array([[0.16, 0, 0, 0], [0, 0.36, 0, 0], [0, 0, 0.16, 0], [0, 0, 0, 0.36]])  # motion noise

    H = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])  # Cartesian observation model
    R = np.array([[0.25, 0], [0, 0.25]])  # observation noise

    x = np.array([[0], [0], [0], [0]])  # initial state
    P = Q  # initial state covariance

    s = np.zeros((4, N))
    abs_errors = []
    squared_errors = []

    for i in range(N):
        xp, Pp = kalmanPredict(x, P, F, Q)
        x, P = kalmanUpdate(xp, Pp, H, R, z[:, i])
        s[:, i] = x.ravel()[:4]
        abs_error = np.sqrt((x[0] - z[0, i])**2 + (x[2] - z[1, i])**2)
        abs_errors.append(abs_error)
        squared_error = abs_error**2
        squared_errors.append(squared_error)

    px = s[0, :]
    py = s[2, :]
    
    mean_abs_error = np.mean(abs_errors)
    std_abs_error = np.std(abs_errors)
    rmse = np.sqrt(np.mean(squared_errors))

    return px, py, mean_abs_error, std_abs_error, rmse

def kalmanPredict(x, P, F, Q):
    xp = np.dot(F, x)  # predict state
    Pp = np.dot(np.dot(F, P), F.T) + Q  # predict state covariance
    return xp, Pp

count = 0
def kalmanUpdate(x, P, H, R, z):
    S = H.dot(P).dot(H.T) + R # innovation covariance
    K = P.dot(H.T).dot(np.linalg.inv(S)) # Kalman gain
    zp = H.dot(x) # predicted observation
    
    global count
    gate = [0,0,0,0,0,0,0,0]
    if gate[count] > 9.21:
        print('Observation outside validation gate')
        xe = x
        Pe = P
        return xe, Pe
    if count == 8:
        return xe, Pe
    count = count+1
    
    xe = x + K.dot(z - zp) # estimated state
    Pe = P - K.dot(S).dot(K.T) # estimated covariance
    
    
    return xe, Pe

# Track the objects
z = np.vstack((a, b))

print(len(z))

px, py , mean_abs_error, std_abs_error, rmse = kalmanTracking(z)
print(f"px: {px}")
print(f"py: {py}")
print(f"Mean Absolute Error: {mean_abs_error}")
print(f"Standard Deviation of Absolute Error: {std_abs_error}")
print(f"Root Mean Squared Error: {rmse}")

print(len(x))
plt.plot(x, y, 'xb')
plt.plot(px, py, '+g')
plt.plot(a, b, '+r')
plt.title('Estimated (magenta) vs Noisy (red) vs Actual (blue)')
plt.legend([ 'Estimated', 'Noisy', 'Actual'], loc='upper left')
plt.show()