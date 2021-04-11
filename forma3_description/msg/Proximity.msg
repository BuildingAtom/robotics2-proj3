# Simple message to for proximity to the robot, output by middleman

Header header       # Reference shenanigans

float64 distance     # Closest distance in meters within valid range (Inf if invalid)
float64 theta        # Approximate theta from the robot frame to the closest distance (Unused, Inf if invalid)
