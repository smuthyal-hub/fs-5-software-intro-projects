import matplotlib.pyplot as plt
from auton_onboard import make_car
from auton_onboard import update
from auton_onboard import calculate_desired_acceleration
from auton_onboard import acceleration_to_throttle_percentage
from auton_onboard import get_gains
 
STEPS = 550
 
car = make_car(desired_v = 40.0, dt = 0.1)

K_P, K_I, K_D = get_gains(car["desired_v"])

velocities, errors, times = [], [], []
for i in range(STEPS):
    accel, error = calculate_desired_acceleration(car, K_P, K_I, K_D)
    throttle = acceleration_to_throttle_percentage(accel)
    update(car, throttle)
    velocities.append(car["v"])
    errors.append(error)
    times.append(car["t"])

fig, (ax1, ax2) = plt.subplots(2, 1, sharex = True)
ax1.plot(times, velocities)
ax1.axhline(car["desired_v"], color ='r', linestyle='--', label = 'Desired Velocity')
ax1.set_ylabel('Velocity (m/s)')
ax2.plot(times, errors)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Error (m/s)')
plt.show()