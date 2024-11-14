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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  //instantiate swerveDriveSubsystem and give it a file path to the json config
  private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  //sendable chooser for selecting auto programs using dashboard
  private final SendableChooser<Command> autoChooser;

  private final int swapAngleButtonID = 10;


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
      () -> processInput(m_driverController.getLeftY(), -Constants.OperatorConstants.TRANSLATION_CURVE, null, OperatorConstants.LEFT_Y_DEADBAND),
      () -> processInput(m_driverController.getLeftX(), -Constants.OperatorConstants.TRANSLATION_CURVE, null, OperatorConstants.LEFT_X_DEADBAND),
      () -> processInput(m_driverController.getRightX(), -Constants.OperatorConstants.ROTATION_CURVE, null, OperatorConstants.RIGHT_Y_DEADBAND),
      () -> processInput(m_driverController.getRightY(), -Constants.OperatorConstants.ROTATION_CURVE, null, OperatorConstants.RIGHT_X_DEADBAND),
      () -> m_driverController.getLeftTriggerAxis()>0.5,
      () -> m_driverController.getRightTriggerAxis()>0.5,
      () -> m_driverController.getLeftBumper(),
      () -> m_driverController.getRightBumper(),
      () -> !m_driverController.getRawButton(swapAngleButtonID)
    );
    Command driveFieldOrientedDirectAngleSim = swerveDriveSubsystem.driveCommand(
      () -> processInput(m_driverController.getRawAxis(1), -Constants.OperatorConstants.TRANSLATION_CURVE, null, OperatorConstants.LEFT_Y_DEADBAND),
      () -> processInput(m_driverController.getRawAxis(0), -Constants.OperatorConstants.TRANSLATION_CURVE, null, OperatorConstants.LEFT_X_DEADBAND),
      () -> processInput(m_driverController.getRawAxis(4), -Constants.OperatorConstants.ROTATION_CURVE, null, OperatorConstants.RIGHT_Y_DEADBAND),
      () -> processInput(m_driverController.getRawAxis(5), -Constants.OperatorConstants.ROTATION_CURVE, null, OperatorConstants.RIGHT_X_DEADBAND),
      () -> m_driverController.getLeftTriggerAxis()>0.5,
      () -> m_driverController.getRightTriggerAxis()>0.5,
      () -> m_driverController.getLeftBumper(),
      () -> m_driverController.getRightBumper(),
      () -> !m_driverController.getRawButton(swapAngleButtonID)
    );

    // left stick controls translation
    // right stick controls the angular velocity of the robot
    // sim command used raw axis for simulating joysticks
    // bumpers and triggers control center of rotation, usefull for evasive meneuvers
    Command driveFieldOrientedAngulerVelocity = swerveDriveSubsystem.driveCommand(
      () -> processInput(m_driverController.getLeftY() , -Constants.OperatorConstants.TRANSLATION_CURVE, null, OperatorConstants.LEFT_Y_DEADBAND),
      () -> processInput(m_driverController.getLeftX() , -Constants.OperatorConstants.TRANSLATION_CURVE, null, OperatorConstants.LEFT_X_DEADBAND),
      () -> processInput(m_driverController.getRightX(), -Constants.OperatorConstants.ROTATION_CURVE, null, OperatorConstants.RIGHT_Y_DEADBAND),
      () -> processInput(m_driverController.getRightY(), -Constants.OperatorConstants.ROTATION_CURVE, null, OperatorConstants.RIGHT_X_DEADBAND),
      () -> m_driverController.getLeftTriggerAxis()>0.5,
      () -> m_driverController.getRightTriggerAxis()>0.5,
      () -> m_driverController.getLeftBumper(),
      () -> m_driverController.getRightBumper(),
      () -> m_driverController.getRawButton(swapAngleButtonID)
    );
    Command driveFieldOrientedAnglularVelocitySim = swerveDriveSubsystem.driveCommand(
      () -> processInput(m_driverController.getRawAxis(1), -Constants.OperatorConstants.TRANSLATION_CURVE, null, OperatorConstants.LEFT_Y_DEADBAND),
      () -> processInput(m_driverController.getRawAxis(0), -Constants.OperatorConstants.TRANSLATION_CURVE, null, OperatorConstants.LEFT_X_DEADBAND),
      () -> processInput(m_driverController.getRawAxis(4), -Constants.OperatorConstants.ROTATION_CURVE, null, OperatorConstants.RIGHT_Y_DEADBAND),
      () -> processInput(m_driverController.getRawAxis(5), -Constants.OperatorConstants.ROTATION_CURVE, null, OperatorConstants.RIGHT_X_DEADBAND),
      () -> m_driverController.getLeftTriggerAxis()>0.5,
      () -> m_driverController.getRightTriggerAxis()>0.5,
      () -> m_driverController.getLeftBumper(),
      () -> m_driverController.getRightBumper(),
      () -> m_driverController.getRawButton(swapAngleButtonID)
    );


    if(Constants.OperatorConstants.ANGULER_VELOCITY){
      // set default command, this command will run until interupted by another command
      // set to sim command when in simulator, this allows propper simulation of inputs
      swerveDriveSubsystem.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAngulerVelocity : driveFieldOrientedAnglularVelocitySim);
    } else {
      swerveDriveSubsystem.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
    }

    // build auto chooser from pathplanner autos
    // add auto chooser to networktables
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    SmartDashboard.putNumber("null", m_driverController.getLeftY());
    SmartDashboard.putNumber("NaN", m_driverController.getLeftX());
  }

  private void configureBindings() {
    //run lock command constantly instead of driver input
    if(RobotBase.isReal()){
      new Trigger(m_driverController::getXButton).whileTrue(Commands.runOnce(swerveDriveSubsystem::lock, swerveDriveSubsystem).repeatedly());
      new Trigger(m_driverController::getYButton).onTrue(Commands.runOnce(swerveDriveSubsystem::zeroGyro, swerveDriveSubsystem));
    } else {
      new Trigger(this::getRawButton3).whileTrue(Commands.runOnce(swerveDriveSubsystem::lock, swerveDriveSubsystem).repeatedly());
      new Trigger(m_driverController::getYButton).onTrue(Commands.runOnce(swerveDriveSubsystem::zeroGyro, swerveDriveSubsystem));
    }
  }
  private boolean getRawButton3(){
    return m_driverController.getRawButton(3);
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
   * 
   * curve function graphed here {@link https://www.desmos.com/calculator/fjuc4iqjqt}
   * @param val - number to process
   * @param multiplier - multiplier for input, mainly used for inverting
   * @param square - polynomial curve value, roughly y=x^s {@link https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html#squaring-inputs}
   * swerve subsystem drive commands square internaly, so this should not be used
   * @param deadZone - deadzone for input {@link https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html#input-deadband}
   * @return
   */
  private double processInput(Double val, Double multiplier, Double square, Double deadZone){
    double out = val;

    if(multiplier != null){out *= multiplier;                           }
    if(square     != null){
      if(deadZone != null && deadZone > 0){
        out  = Math.signum(val) * Math.pow((1/(-deadZone+1))*Math.abs(val)-(deadZone/(-deadZone+1)), square);
      } else {
        out  = Math.signum(val) * Math.pow(Math.abs(val), square);
      }
    }
    if(deadZone   != null){out  = MathUtil.applyDeadband(out, deadZone);}

    return out;
  }
}
