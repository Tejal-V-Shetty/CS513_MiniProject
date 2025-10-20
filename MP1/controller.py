"""This file is the main controller file.

Here, you will design the controller for your for the adaptive cruise control system.
"""

from mp1_simulator.simulator import Observation, Mode

from typing import Tuple


# NOTE: Very important that the class name remains the same
class Controller:
    def __init__(self, distance_threshold: float):
        self.distance_threshold = distance_threshold

    def run_step(self, obs: Observation) -> Tuple[float, Mode]:
        """This is the main run step of the controller.

        Here, you will have to read in the observations `obs`, process it, and output an
        acceleration value and the operation mode.
        
        The acceleration value must be some value between -10.0 and 10.0.

        For the operation mode output True when "following" and False when "cruising".

        Note that the acceleration value is really some control input that is used
        internally to compute the throttle an brake values of the car.

        Below is some example code where the car just outputs the control value 10.0
        """

        ego_velocity = obs.ego_velocity
        desired_speed = obs.desired_speed
        dist_to_lead = obs.distance_to_lead
        
        target_speed = desired_speed
        accel_val = 1.0
        mode_val = Mode.CRUISING
        epsilon = 0.1
        # Do your magic...
        factor = 10
        safety_lim = ego_velocity*factor/2  #Modify limit to start braking based on speed of car
        if dist_to_lead<safety_lim*self.distance_threshold:   #Brake if close to lead car
            accel_val = -5.0 * (safety_lim * self.distance_threshold - dist_to_lead) / self.distance_threshold
            mode_val = Mode.FOLLOWING
            return(accel_val, mode_val)
        
        if ego_velocity<desired_speed+epsilon:  #If not at the target_speed
            accel_val = (desired_speed-ego_velocity)*10.0
            #print("Target: ", target_speed, " Accel: ", accel_val)
        
        if dist_to_lead<=factor*self.distance_threshold:    #Slow down as you approach the lead
            target_speed = desired_speed*(dist_to_lead/(factor*self.distance_threshold))
            dist_accel = (dist_to_lead-factor*self.distance_threshold)/self.distance_threshold  #Acceleration calculated based on distance threshold
            speed_accel = (target_speed-ego_velocity)*10.0  #Acceleration calculated based on speed threshold
            alpha = min(1.0, max(0.0, (factor*self.distance_threshold - dist_to_lead) / ((factor-1)*self.distance_threshold)))
            accel_val = alpha*dist_accel+(1-alpha)*speed_accel
            mode_val = Mode.FOLLOWING

        if accel_val>10.0:
            accel_val = 10.0
        elif accel_val<-10.0:
            accel_val = -10.0
            
        return (accel_val, mode_val)
