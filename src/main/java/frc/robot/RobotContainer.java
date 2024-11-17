package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.utils.ExtraMath;

import java.io.File;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {


  private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  private final SendableChooser<Command> autoChooser;

  private boolean fod = Constants.drive.STARTING_FOD;
  private boolean directAngle = Constants.drive.STARTING_DIRECT_ANGLE;

  private final XboxController m_driverController =
      new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  private PowerDistribution pdp = new PowerDistribution();

  public RobotContainer() {
    configureBindings();

    SmartDashboard.putData("PDP", pdp);

    
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    // sim command used raw axis for simulating joysticks
    // bumpers and triggers control center of rotation, usefull for evasive meneuvers
    Command driveAngulerVelocity = swerveDriveSubsystem.driveCommand(
      () -> ExtraMath.processInput(m_driverController.getLeftY() , -ExtraMath.remap(m_driverController.getLeftTriggerAxis() , 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.TRANSLATION_CURVE, OperatorConstants.LEFT_Y_DEADBAND),
      () -> ExtraMath.processInput(m_driverController.getLeftX() , -ExtraMath.remap(m_driverController.getLeftTriggerAxis() , 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.TRANSLATION_CURVE, OperatorConstants.LEFT_X_DEADBAND),
      () -> ExtraMath.processInput(m_driverController.getRightX(), -ExtraMath.remap(m_driverController.getRightTriggerAxis(), 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.ROTATION_CURVE, OperatorConstants.RIGHT_X_DEADBAND),
      () -> ExtraMath.processInput(m_driverController.getRightY(), -ExtraMath.remap(m_driverController.getRightTriggerAxis(), 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.ROTATION_CURVE, OperatorConstants.RIGHT_X_DEADBAND),
      () -> this.getDirectAngle(),
      () -> this.getFOD()
    );
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    // sim command used raw axis for simulating joysticks
    // bumpers and triggers control center of rotation, usefull for evasive meneuvers
    Command driveAngulerVelocitySim = swerveDriveSubsystem.driveCommand(
      () -> ExtraMath.processInput(m_driverController.getRawAxis(1), -ExtraMath.remap(m_driverController.getRawAxis(2), 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.TRANSLATION_CURVE, OperatorConstants.LEFT_Y_DEADBAND),
      () -> ExtraMath.processInput(m_driverController.getRawAxis(0), -ExtraMath.remap(m_driverController.getRawAxis(2), 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.TRANSLATION_CURVE, OperatorConstants.LEFT_X_DEADBAND),
      () -> ExtraMath.processInput(m_driverController.getRawAxis(4), -ExtraMath.remap(m_driverController.getRawAxis(3), 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.ROTATION_CURVE, OperatorConstants.RIGHT_X_DEADBAND),
      () -> ExtraMath.processInput(m_driverController.getRawAxis(5), -ExtraMath.remap(m_driverController.getRawAxis(3), 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.ROTATION_CURVE, OperatorConstants.RIGHT_X_DEADBAND),
      () -> this.getDirectAngle(),
      () -> this.getFOD()
    );

    swerveDriveSubsystem.setDefaultCommand(RobotBase.isReal() ? driveAngulerVelocity : driveAngulerVelocitySim);
    

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    //run lock command constantly instead of driver input
    if(RobotBase.isReal()){
      new Trigger(m_driverController::getXButton).whileTrue(Commands.runOnce(swerveDriveSubsystem::lock, swerveDriveSubsystem).repeatedly());
      new Trigger(m_driverController::getAButton).onTrue(Commands.runOnce(swerveDriveSubsystem::zeroGyro, swerveDriveSubsystem));
      new Trigger(m_driverController::getLeftStickButton).onTrue(Commands.runOnce(() -> {this.fod = !this.fod;}));
      new Trigger(m_driverController::getRightStickButton).onTrue(Commands.runOnce(() -> {this.directAngle = !this.directAngle;}));
    } else {
      new Trigger(() -> m_driverController.getRawButton(3)).whileTrue(Commands.runOnce(swerveDriveSubsystem::lock, swerveDriveSubsystem).repeatedly());
      new Trigger(() -> m_driverController.getRawButton(1)).onTrue(Commands.runOnce(swerveDriveSubsystem::zeroGyro, swerveDriveSubsystem));
      new Trigger(() -> m_driverController.getRawButton(9)).onTrue(Commands.runOnce(() -> {this.fod = !this.fod;}));
      new Trigger(() -> m_driverController.getRawButton(10)).onTrue(Commands.runOnce(() -> {this.directAngle = !this.directAngle;}));
    }
  }

  public void Periodic(){
    SmartDashboard.putBoolean("fod", getFOD());
    SmartDashboard.putBoolean("direct angle", directAngle);
  }
  public void SimPeriodic(){
  }

  public Command getAutonomousCommand() {
    Command auto = autoChooser.getSelected();
    return auto;
  }

  
  public boolean getFOD(){return fod;}
  public boolean getDirectAngle(){return directAngle;}

}
