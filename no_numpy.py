import random
import time
from math import sin, cos, radians

# Define state vector with position, velocity, heading, pitch, and 2D acceleration
x = [0, 0, 1, 0, 0, 0, 0]

# Process noise covariance matrix
Q = [0.5, 0.5, 0.05, 0.05, 0.1, 0.2, 0.2]  # Adjust variances as needed

# Measurement noise covariance matrix
R = [64, 64]  # Adjust for GPS and heading uncertainty

P = [[100 if i == j else 0 for j in range(7)] for i in range(7)]  # Initial covariance matrix

def state_transition(x, dt, wheel_speed, heading, ax, ay, pitch):
  """
  Updates the state vector based on motion and pitch.

  Args:
    x: Current state vector.
    dt: Time since last update.
    wheel_speed: Measured wheel speed.
    heading: Measured heading.
    ax: Measured x-axis acceleration.
    ay: Measured y-axis acceleration.
    pitch: Estimated pitch angle.

  Returns:
    Updated state vector.
  """

  x[5] = ax*cos(pitch)
  x[6] = ay*cos(pitch)

  vx = (wheel_speed*cos(heading) + x[5] * dt)
  vy = (wheel_speed*sin(heading) + x[6] * dt)


  # Update position and velocity


  x[2] = vx*cos(pitch)
  x[3] = vy*cos(pitch)

  sx = (x[2]*dt + 0.5*x[5]*dt**2)
  sy = (x[3]*dt + 0.5*x[6]*dt**2)

  x[0] += sx*cos(pitch)
  x[1] += sy*cos(pitch)

  # Integrate acceleration for next prediction


  return x

def measurement(x):
    measured_x = x[0]
    measured_y = x[1]
    return [measured_x, measured_y]

def update_R(dt_gps):
    updated_R = [r * (2 ** dt_gps) for r in R]  # Adjust the scaling factor as needed
    return updated_R

def predict(x, dt, wheel_speed, heading, ax, ay, pitch, P):
    F = [
        [1, 0, dt * cos(heading), 0, -dt * sin(pitch) * ax, 0, 0],
        [0, 1, dt * sin(heading), 0, dt * cos(pitch) * ay, 0, 0],
        [0, 0, 1, 0, -wheel_speed * sin(heading) * cos(pitch), 0, 0],
        [0, 0, 0, 1, wheel_speed * cos(heading) * cos(pitch), 0, 0],
        [0, 0, 0, 0, 1, 0, 0],
        [dt * sin(pitch) * ax, dt * cos(pitch) * ay, 0, 0, 0, 1, 0],
        [0, dt, 0, 0, 0, 0, 1]
    ]
    x_pred = state_transition(x, dt, wheel_speed, heading, ax, ay, pitch)
    P_pred = [
        [sum(F[i][k] * P[k][j] for k in range(7)) for j in range(7)] for i in range(7)
    ]
    for i in range(7):
        for j in range(7):
            P_pred[i][j] += Q[i] * F[i][j]
    return x_pred, P_pred

def update(x_pred, P_pred, gps_data, dt_gps):
    updated_R = update_R(dt_gps)
    H = [
        [1, 0, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0, 0]
    ]
    z = gps_data  # 2D GPS measurement
    innovation = [z[i] - measurement(x_pred)[i] for i in range(2)]
    
    print(innovation)
    
    S = [
        [sum(H[i][k] * P_pred[k][j] for k in range(7)) for j in range(7)] for i in range(2)
    ]
    for i in range(2):
        for j in range(7):
            S[i][j] += updated_R[i]

    # Calculate the determinant of S
    det_S = S[0][0] * S[1][1] - S[0][1] * S[1][0]

    # Calculate the Kalman gain
    K = [
        [sum(H[i][k] * P_pred[k][j] * det_S ** -1 * innovation[j % 2] for k in range(min(2, len(P_pred[0])))) for j in range(min(2, len(P_pred[0])))]
        for i in range(7)
    ]




    # Update state and covariance matrix
    x = [x_pred[i] + sum(K[i][j] * innovation[j] for j in range(2)) for i in range(7)]
    P = [
        [P_pred[i][j] - sum(K[i][j] * H[i][k] * P_pred[k][j] for k in range(7)) for j in range(7)] for i in range(7)
    ]
    return x, P






# Replace with actual sensor data and simulation loop
# ...

# Main loop
timer = 0
timer1 = 0
gps_x = 0
gps_y = 0
x_points = []
y_points = []
xgps = []
ygps = []
heading_arr = []
sx = 0
sy = 0
state = True
time_rn = time.time()
last_gps_update = time.time()

for i in range(1000):
    dt = time.time() - last_gps_update  # Calculate time since last GPS
    last_gps_update = time.time()
    timer += dt
    wheel_speed = random.uniform(0.95, 1.05)
    heading = radians(random.uniform(-0.05, 0.05))
    pitch = radians(random.uniform(-0.05, 0.05))
    heading_arr.append(heading)
    ax = random.uniform(-0.05, 0.05)
    ay = random.uniform(-0.05, 0.05)
    x_pred, P_pred = predict(x, dt, wheel_speed, heading, ax, ay, pitch, P)
    sx += ((wheel_speed * cos(heading) + ax * dt) * dt + 0.5 * ax * dt ** 2) * cos(pitch)
    sy += ((wheel_speed * sin(heading) + ay * dt) * dt + 0.5 * ay * dt ** 2) * cos(pitch)
    x, P = update(x_pred, P_pred, [gps_x, gps_y], timer)
    if timer > 1:
        gps_x = sx + random.uniform(-0.5, 0.5)
        gps_y = sy + random.uniform(-0.5, 0.5)
        timer = 0
    time.sleep(0.5)
    print(x[0], x[1])
