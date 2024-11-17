package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.utils.ExtraMath;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private double autoStart = 0.0;
  private double teleopStart = 0.0;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    DataLogManager.start();
    m_robotContainer = new RobotContainer();

    if(!RobotBase.isSimulation()){
      DataLogManager.start();
    }
    SmartDashboard.putNumber("time/upTime", 0.0);

    SmartDashboard.putNumber("time/auto/StartTime"    , -1.0);
    SmartDashboard.putNumber("time/auto/UpTime"       , -1.0);
    SmartDashboard.putNumber("time/auto/RemainingTime", -1.0);
    SmartDashboard.putNumber("time/auto/TotalTime"    , Constants.AUTO_TIME);
    
    SmartDashboard.putNumber("time/teleop/StartTime"    , -1.0);
    SmartDashboard.putNumber("time/teleop/UpTime"       , -1.0);
    SmartDashboard.putNumber("time/teleop/RemainingTime", -1.0);
    SmartDashboard.putNumber("time/teleop/TotalTime"    , Constants.TELEOP_TIME);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("time/upTime", Timer.getFPGATimestamp());

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    autoStart = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("time/auto/StartTime", autoStart);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("time/auto/UpTime"       , Timer.getFPGATimestamp()-autoStart);
    SmartDashboard.putNumber("time/auto/RemainingTime", ExtraMath.holdPositive(Constants.AUTO_TIME-(Timer.getFPGATimestamp()-autoStart)));
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    teleopStart = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("time/teleop/StartTime", teleopStart);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("time/teleop/UpTime"       , Timer.getFPGATimestamp()-teleopStart);
    SmartDashboard.putNumber("time/teleop/RemainingTime", ExtraMath.holdPositive(Constants.TELEOP_TIME-(Timer.getFPGATimestamp()-teleopStart)));
    m_robotContainer.Periodic();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    m_robotContainer.SimPeriodic();

  }
}
