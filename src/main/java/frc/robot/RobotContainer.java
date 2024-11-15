package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.utils.ExtraMath;

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


  private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  private final SendableChooser<Command> autoChooser;

  private final XboxController m_driverController =
      new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  public RobotContainer() {
    configureBindings();

    
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    // sim command used raw axis for simulating joysticks
    // bumpers and triggers control center of rotation, usefull for evasive meneuvers
    Command driveRobotOrientedAngulerVelocity = swerveDriveSubsystem.driveCommand(
      () -> ExtraMath.processInput(m_driverController.getLeftY() , -1.0, Constants.OperatorConstants.TRANSLATION_CURVE, OperatorConstants.LEFT_Y_DEADBAND),
      () -> ExtraMath.processInput(m_driverController.getLeftX() , -1.0, Constants.OperatorConstants.TRANSLATION_CURVE, OperatorConstants.LEFT_X_DEADBAND),
      () -> ExtraMath.processInput(m_driverController.getRightX(), -1.0, Constants.OperatorConstants.ROTATION_CURVE, OperatorConstants.RIGHT_X_DEADBAND)
    );

    swerveDriveSubsystem.setDefaultCommand(driveRobotOrientedAngulerVelocity);
    

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    //run lock command constantly instead of driver input
    if(RobotBase.isReal()){
      new Trigger(m_driverController::getXButton).whileTrue(Commands.runOnce(swerveDriveSubsystem::lock, swerveDriveSubsystem).repeatedly());
      new Trigger(m_driverController::getYButton).onTrue(Commands.runOnce(swerveDriveSubsystem::zeroGyro, swerveDriveSubsystem));
    } else {
      new Trigger(() -> m_driverController.getRawButton(3)).whileTrue(Commands.runOnce(swerveDriveSubsystem::lock, swerveDriveSubsystem).repeatedly());
      new Trigger(m_driverController::getYButton).onTrue(Commands.runOnce(swerveDriveSubsystem::zeroGyro, swerveDriveSubsystem));
    }
  }

  public Command getAutonomousCommand() {
    Command auto = autoChooser.getSelected();
    return auto;
  }

}
