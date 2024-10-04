package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.io.File;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  //instantiate swerveDriveSubsystem and give it a file path to the json config
  private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  //sendable chooser for selecting auto programs using dashboard
  private final SendableChooser<Command> autoChooser;

  //driver Xbox controller, handles input
  private final XboxController m_driverController =
      new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  public RobotContainer() {
    configureBindings();//configure controller bindings. command-based programming defines triggers that are called on events
                        // for example, to run a command when a button is pressed, you need to define a trigger 
                        // 
                        //Trigger button = new Trigger(m_driverController.getAButton());
                        // button.onTrue([Command]);

    
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    // sim command used raw axis for simulating joysticks
    // bumpers and triggers control center of rotation, usefull for evasive meneuvers
    Command driveFieldOrientedDirectAngle = swerveDriveSubsystem.driveCommand(
      () -> processInput(m_driverController.getLeftY(), -1.0, null, OperatorConstants.LEFT_Y_DEADBAND),
      () -> processInput(m_driverController.getLeftX(), -1.0, null, OperatorConstants.LEFT_X_DEADBAND),
      () -> processInput(m_driverController.getRawAxis(4), -1.0, null, OperatorConstants.RIGHT_Y_DEADBAND),
      () -> processInput(m_driverController.getRawAxis(5), -1.0, null, OperatorConstants.RIGHT_X_DEADBAND),
      () -> m_driverController.getLeftTriggerAxis()>0.5,
      () -> m_driverController.getRightTriggerAxis()>0.5,
      () -> m_driverController.getLeftBumper(),
      () -> m_driverController.getRightBumper()
    );
    Command driveFieldOrientedDirectAngleSim = swerveDriveSubsystem.driveCommand(
      () -> processInput(m_driverController.getRawAxis(1), -1.0, null, OperatorConstants.LEFT_Y_DEADBAND),
      () -> processInput(m_driverController.getRawAxis(0), -1.0, null, OperatorConstants.LEFT_X_DEADBAND),
      () -> processInput(m_driverController.getRawAxis(4), -1.0, null, OperatorConstants.RIGHT_Y_DEADBAND),
      () -> processInput(m_driverController.getRawAxis(5), -1.0, null, OperatorConstants.RIGHT_X_DEADBAND),
      () -> m_driverController.getLeftTriggerAxis()>0.5,
      () -> m_driverController.getRightTriggerAxis()>0.5,
      () -> m_driverController.getLeftBumper(),
      () -> m_driverController.getRightBumper()
    );

    // left stick controls translation
    // right stick controls the angular velocity of the robot
    // sim command used raw axis for simulating joysticks
    // bumpers and triggers control center of rotation, usefull for evasive meneuvers
    Command driveFieldOrientedAnglularVelocity = swerveDriveSubsystem.driveAVCommand(
      () -> processInput(m_driverController.getLeftY(), -1.0, null, OperatorConstants.LEFT_Y_DEADBAND),
      () -> processInput(m_driverController.getLeftX(), -1.0, null, OperatorConstants.LEFT_X_DEADBAND),
      () -> processInput(m_driverController.getRightX(), -1.0, null, OperatorConstants.RIGHT_X_DEADBAND),
      () -> m_driverController.getLeftTriggerAxis()>0.5,
      () -> m_driverController.getRightTriggerAxis()>0.5,
      () -> m_driverController.getLeftBumper(),
      () -> m_driverController.getRightBumper()
    );
    Command driveFieldOrientedAngulerVelocitySim = swerveDriveSubsystem.driveAVCommand(
      () -> processInput(m_driverController.getRawAxis(1), -1.0, null, OperatorConstants.LEFT_Y_DEADBAND),
      () -> processInput(m_driverController.getRawAxis(0), -1.0, null, OperatorConstants.LEFT_X_DEADBAND),
      () -> processInput(m_driverController.getRawAxis(4), -1.0, null, OperatorConstants.RIGHT_X_DEADBAND),
      () -> m_driverController.getLeftTriggerAxis()>0.5,
      () -> m_driverController.getRightTriggerAxis()>0.5,
      () -> m_driverController.getLeftBumper(),
      () -> m_driverController.getRightBumper()
    );


    if(Constants.OperatorConstants.ANGULER_VELOCITY){
      // set default command, this command will run until interupted by another command
      // set to sim command when in simulator, this allows propper simulation of inputs
      swerveDriveSubsystem.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedAngulerVelocitySim);
    } else {
      swerveDriveSubsystem.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
    }

    // build auto chooser from pathplanner autos
    // add auto chooser to networktables
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return selected auto from networktables
    Command auto = autoChooser.getSelected();
    return auto;
  }

  /**
   * process input value
   * @param val - number to process
   * @param multiplier - multiplier for input, mainly used for inverting
   * @param square - polynomial curve value, roughly y=x^s {@link https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html#squaring-inputs}
   * @param deadBand - deadband for input {@link https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html#input-deadband}
   * @return
   */
  private double processInput(Double val, Double multiplier, Double square, Double deadBand){
    double out = val;

    if(multiplier != null){out *= multiplier;                           }
    if(square     != null){out  = square(out, square);                  }
    if(deadBand   != null){out  = MathUtil.applyDeadband(out, deadBand);}

    return out;
  }
  private double square(double val, double mag){
    return Math.signum(val) * Math.pow(Math.abs(val), mag);
  }
}
