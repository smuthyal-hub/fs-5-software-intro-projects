import numpy as np


def make_car(desired_v:float=20.0, dt:float=0.1) -> dict:
    """ 
    Generates a dictionary that holds all the car's values. Keeps track of state varaibles.
    """
    car_state_dictionary : dict[str, float] = {
        "v" : 0, #velocity of your car 
        "a" : 0, #acceleration of your car
        "t" : 0, #time of your car
        "x" : 0, #position of your car
        "dt" : dt, #time step of your car, how much the time changes every time you update/step
        "desired_v" : desired_v, #desired velocity of your car, the velocity you want to maintain
        "step" : 0,
    
        #hint: use these variables in the integral and derivative portion of your PID control (steps 5 and 6 )
        "error_prev" : None,
        "net_integral" : 0.0
    }
    return car_state_dictionary

def update(car: dict, throttle_perc: float, mass: float = 1000, max_throttle_force: float = 5000, friction: float = 2.0) -> None:
        """
        Updates the car's state variables based on the throttle percentage.
        Use this function after finding throttle percentage to update the car's state variables.

        Inputs:
        car: dictionary containing the car's state variables
        throttle_perc: float, throttle percentage (-1 to 1)

        Outputs:
        None, but updates the car's state variables
        """
        force = throttle_perc * max_throttle_force
        car["a"] = (force / mass) - friction
        car["v"] += car["a"] * car["dt"]
        car["x"] += car["v"] * car["dt"]
        car["t"] += car["dt"]
        car["step"] += 1


def calculate_desired_acceleration(car, K_P, K_I = 0.0, K_D = 0.0) :
        dt = car["dt"]
        error = car["desired_v"] - car["v"]
        car["net_integral"] += error * dt
        if K_I > 0:
            limit = 5.0/K_I
            car["net_integral"] = max(-limit, min(limit, car["net_integral"]))
        if car["error_prev"] is None:
            derivative = 0.0
        else:
            derivative = (error - car["error_prev"]) / dt
        car["error_prev"] = error

        desired_acceleration = K_P * error + K_I * car["net_integral"] + K_D * derivative
        return desired_acceleration, error

def get_gains(desired_v: float) -> tuple[float, float, float]:
    if desired_v < 15:
        return 0.5, 0.05, 0.3
    else:
        return 1.0, 0.1, 0.3

def acceleration_to_throttle_percentage(acceleration_desired: float, mass: float = 1000, max_throttle_force: float = 5000) -> float:
        #input: desired_acceleration(float)
        #output: throttle percentage (float, -1 to 1)
        max_acceleration = max_throttle_force / mass
        throttle_percentage = acceleration_desired / max_acceleration
        return float(np.clip(throttle_percentage, -1.0, 1.0))