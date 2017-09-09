import numpy as np

# x Prevent the direct going into the rocks
# right wall crawling


def decision_step(Rover):

    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:

                    # condition for stuck code
                    if Rover.vel < 0.1 and Rover.vel > -1.0:
                        Rover.stuck_time_threshold += 1
                        Rover.throttle = Rover.throttle_set
                        if Rover.stuck_time_threshold > 100:
                            Rover.stuck = True
                        if Rover.stuck:
                            Rover.throttle = 0
                            # Set brake to stored brake value
                            Rover.brake = Rover.brake_set
                            Rover.steer = 0
                            Rover.mode = 'stop'
                            

                        # Set throttle value to throttle setting
                    else:
                        Rover.stuck_time_threshold = 0
                        Rover.stuck = False
                        Rover.throttle = Rover.throttle_set

                    # to make the throttle proportional to navigable area
                    # Rover.throttle = len(Rover.nav_angles) / 100
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0

                # Set steering to average angle clipped to the range +/- 15
                # for wall crawling
                steer_offset = -15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi + steer_offset), -15, 15)
                Rover.data = Rover.steer
                
                # Rover.data = np.mean(Rover.nav_angles * 180/np.pi )
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward or Rover.stuck:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    print("turning")
                    Rover.steer = -15
                    
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward and (np.mean(Rover.nav_angles * 180/np.pi) < 4.10 and np.mean(Rover.nav_angles * 180/np.pi) > -4.10 ):
                    # Set throttle back to stored value
                    Rover.stuck = False
                    Rover.stuck_time_threshold = 0
                    
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle

                    # for the wall crawling/following
                    steer_offset = -15
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi + steer_offset), -15, 15)
                    Rover.data = Rover.steer
                    # Rover.data = np.mean(Rover.nav_angles * 180/np.pi )
                    Rover.mode = 'forward'

    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    
        
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

