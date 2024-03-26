#!/usr/bin/env python3
import rospy

class PIDController():
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_prior = 0
        self.error_prior = 0
    
    def update(self, setpoint, measured_value, dt):
        '''
        Update the PID Controller
        
        :param setpoint: Desired target value
        :param measured_value: Current value
        :param dt: Time interval since  the last update
        :return: Control output
        '''
        error = setpoint - measured_value
        integral = self.integral_prior + error * dt
        derivative = (error - self.error_prior)
        output = self.kp * error + self.ki * integral + self.kd * derivative
        
        self.error_prior = error
        self.integral_prior = integral
        
        return output
    

'''if __name__ == '__main__':
    #bottle_weight = 10
    #initial_measurement = 20
    #desired_spice_amount = 2
    
    pid = PIDController(1, 0, 0)
    print(pid.update())
    
    while not rospy.is_shutdown():'''
        