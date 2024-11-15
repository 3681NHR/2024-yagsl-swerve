# 2024-yagsl-swerve

## what does what?!?!?!?!

- Main.java - dont touch
- Robot.java - rarely used
- RobotContainer.java - robot init and trigger bindings
handles bindings and input
- Constants.java - constants and cool numbers
- subsystems
    - Vision.java - handles PhotonVision
    - SwerveDriveSubsystem.java - handles swerve drive

## notes

**driver controller is port 0, assign controller or keyboard to port 0 in order to control**

 - when in simulator, swerve takes raw buttons and axis for input, this allows simulation to work with both xbox controller and keyboard
 - remember us check "map gamepad" in simulatior if using controleler
 - using keyboard in simulator requires sim GUI window to have focus
 - to connect dashboards to simulator
    - advantagescope has a button for this
    - others need to set the ip to "localhost"
 - angle setpoins are being changed by 180 degrees everytime the wheel starts to turn. prob something with optimisation


## todo
- auto aim buttons
- fix gyro