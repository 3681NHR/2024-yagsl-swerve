package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.utils.rumble.ControllerRumble;
import frc.utils.rumble.AddRumbleType;
import frc.utils.TimerHandler;
import frc.utils.ExtraMath;
import frc.utils.rumble.RumblePreset;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
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

  private boolean fod = Constants.drive.STARTING_FOD;
  private boolean directAngle = Constants.drive.STARTING_DIRECT_ANGLE;

  private Trigger lockPose;
  private Trigger rstGyro;

  private final XboxController m_driverController =
      new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  private ControllerRumble rumbler = new ControllerRumble(m_driverController);

  private PowerDistribution pdp = new PowerDistribution();

  public RobotContainer() {
    SmartDashboard.putBoolean("Vision enabled", true);
    configureBindings();

    SmartDashboard.putData("PDP", pdp);

    // left stick controls translation
    // right stick controls the angular velocity of the robot
    //direct angle selects if right stick should control robot angle
    //fod selects if using field orientation
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
    // sim command uses raw axis for simulating joysticks
    //direct angle selects if right stick should control robot angle
    //fod selects if using field orientation
    Command driveAngulerVelocitySim = swerveDriveSubsystem.driveCommand(
      () -> ExtraMath.processInput(m_driverController.getRawAxis(1), -ExtraMath.remap(m_driverController.getRawAxis(2), 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.TRANSLATION_CURVE, OperatorConstants.LEFT_Y_DEADBAND),
      () -> ExtraMath.processInput(m_driverController.getRawAxis(0), -ExtraMath.remap(m_driverController.getRawAxis(2), 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.TRANSLATION_CURVE, OperatorConstants.LEFT_X_DEADBAND),
      () -> ExtraMath.processInput(m_driverController.getRawAxis(4), -ExtraMath.remap(m_driverController.getRawAxis(3), 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.ROTATION_CURVE, OperatorConstants.RIGHT_X_DEADBAND),
      () -> ExtraMath.processInput(m_driverController.getRawAxis(5), -ExtraMath.remap(m_driverController.getRawAxis(3), 0.0, 1.0, 1.0, 0.1), Constants.OperatorConstants.ROTATION_CURVE, OperatorConstants.RIGHT_X_DEADBAND),
      () -> this.getDirectAngle(),
      () -> this.getFOD()
    );

    //set default drive command
    swerveDriveSubsystem.setDefaultCommand(RobotBase.isReal() ? driveAngulerVelocity : driveAngulerVelocitySim);
    
    //build and publish auto chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * generate triggers for keybinding
   */
  private void configureBindings() {
    //reset odometry button
    SmartDashboard.putBoolean("reset odometry", false);
    new Trigger(() ->SmartDashboard.getBoolean("reset odometry", false)).onTrue(Commands.runOnce(() -> {
      SmartDashboard.putBoolean("reset odometry", false);
      swerveDriveSubsystem.resetOdometry(new Pose2d());
    }));

    new Trigger(() -> SmartDashboard.getBoolean("Vision enabled", false)).onTrue(Commands.runOnce(() -> swerveDriveSubsystem.setVision(true)));
    new Trigger(() -> SmartDashboard.getBoolean("Vision enabled", false)).onFalse(Commands.runOnce(() -> swerveDriveSubsystem.setVision(false)));

    //bindings depend of if using simulation
    if(RobotBase.isReal()){
      lockPose = new Trigger(m_driverController::getXButton);
      rstGyro = new Trigger(m_driverController::getAButton);
      new Trigger(m_driverController::getLeftStickButton).onTrue(Commands.runOnce(() -> {this.fod = !this.fod;}));
      new Trigger(m_driverController::getRightStickButton).onTrue(Commands.runOnce(() -> {this.directAngle = !this.directAngle;}));
      new Trigger(m_driverController::getYButton).onTrue(Commands.runOnce(() -> rumbler.addRumble(RumblePreset.RING.load(), AddRumbleType.OVERLAY)));
    } else {
      lockPose = new Trigger(() -> m_driverController.getRawButton(3));
      rstGyro = new Trigger(() -> m_driverController.getRawButton(1));
      new Trigger(() -> m_driverController.getRawButton(9)).onTrue(Commands.runOnce(() -> {this.fod = !this.fod;}));
      new Trigger(() -> m_driverController.getRawButton(10)).onTrue(Commands.runOnce(() -> {this.directAngle = !this.directAngle;}));
      
      
    }
      //lockpose binding
      lockPose.whileTrue(Commands.runOnce(swerveDriveSubsystem::lock, swerveDriveSubsystem).repeatedly());
      //reset gyro binding
      rstGyro.onTrue(Commands.runOnce(swerveDriveSubsystem::zeroGyro, swerveDriveSubsystem));

      //rumble bindings for lockpose and reset gyro
      //kl;lockPose.whileTrue(Commands.runOnce(() -> rumbler.addRumble(0.1, 0.1, AddRumbleType.OVERLAY)).repeatedly());
      //rstGyro.onTrue(Commands.runOnce(() -> rumbler.addRumble(RumblePreset.TAP.load(), AddRumbleType.OVERLAY)));

      //rumble binding when time remaining is low
      new Trigger(() -> TimerHandler.getTeleopRemaining()<30.0).or(
        new Trigger(() -> TimerHandler.getAutoRemaining()<3.0)
      ).onTrue(Commands.runOnce(() -> {
        rumbler.addRumble(RumblePreset.DOUBLE_TAP.load(), AddRumbleType.OVERRIDE);
      }));
  }

  public void Periodic(){
    SmartDashboard.putBoolean("fod", getFOD());
    SmartDashboard.putBoolean("direct angle", directAngle);

    rumbler.update();
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
