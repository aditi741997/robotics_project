define minirobot position
(
    color "blue"          # Default color.
    drive "diff"         # Differential steering model.
    gui_nose 1           # Draw a nose on the robot so we can see which way it points
    obstacle_return 1    # Can hit things.
    laser_return 1                  # reflects sonar beams


    # alternative odometric localization with simple error model
    localization "gps"                 # Change to "gps" to have impossibly perfect, global odometry

    # Actual size
    size [0.4 0.4 0.4]

    # The pioneer's center of rotation is offset from its center of area
    origin [0 0 0 0]
)