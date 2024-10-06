package frc.robot.subsystems;

import frc.robot.Constants;

import java.io.File;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/** swervedrive using YAGSL, this allows configuration using json files
 * YAGSL also handles kinematics and odometry for the drive
 */
public class SwerveDriveSubsystem extends SubsystemBase {
  
  private final SwerveDrive drive;

  private       Vision  vision;
  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private final boolean visionEnabled = true;

  /**
   * YAGSL swerve subsystem constructor
   * @param directory - directory of YAGSL config json
   */
  public SwerveDriveSubsystem(File directory) {
    
    // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    //  In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8);
    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER RESOLUTION).
    //  In this case the wheel diameter is 4 inches, which must be converted to meters to get meters/second.
    //  The gear ratio is 6.75 motor revolutions per wheel rotation.
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75);
    System.out.println("\"conversionFactors\": {");
    System.out.println("\t\"angle\": {\"factor\": " + angleConversionFactor + " },");
    System.out.println("\t\"drive\": {\"factor\": " + driveConversionFactor + " }");
    System.out.println("}");
    
    try
    {
      drive = new SwerveParser(directory).createSwerveDrive(Constants.drive.MAX_SPEED);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }

    drive.setCosineCompensator(!RobotBase.isSimulation());

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    setupPathPlanner();
    if (visionEnabled)
    {
      setupPhotonVision();
      // Stop the odometry thread if we are using vision that way we can synchronize updates better.
      drive.stopOdometryThread();
    }
  }

  

  @Override
  public void periodic() {
    // When vision is enabled we must manually update odometry in SwerveDrive
    if (visionEnabled)
    {
      drive.updateOdometry();
      vision.updatePoseEstimation(drive);
    }
  }

  @Override
  public void simulationPeriodic() {
    
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY)
  {
    return run(() -> {
      drive.setHeadingCorrection(true);

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
        translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(drive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
        headingX.getAsDouble(),
        headingY.getAsDouble(),
        drive.getOdometryHeading().getRadians(),
        drive.getMaximumVelocity()));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      drive.setHeadingCorrection(false);
      // Make the robot move
      drive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            translationX.getAsDouble() * drive.getMaximumVelocity(),
                            translationY.getAsDouble() * drive.getMaximumVelocity()), 0.8),
                        Math.pow(angularRotationX.getAsDouble(), 3) * drive.getMaximumAngularVelocity(),
                        true,
                        false);
    });
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * 
   * fl, fr, bl, br - boolSupplier for rotation point buttons
   * 
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY, BooleanSupplier fl, BooleanSupplier fr, BooleanSupplier bl, BooleanSupplier br)
  {
    // drive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
        translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(drive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
        headingX.getAsDouble(),
        headingY.getAsDouble(),
        drive.getOdometryHeading().getRadians(),
        drive.getMaximumVelocity()),
        getPivot(fl, fr, bl, br));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @return Drive command.
   */
  public Command driveAVCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX, BooleanSupplier fl, BooleanSupplier fr, BooleanSupplier bl, BooleanSupplier br)
  {
    return run(() -> {
      // Make the robot move
      drive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            translationX.getAsDouble() * drive.getMaximumVelocity(),
                            translationY.getAsDouble() * drive.getMaximumVelocity()), 0.8),
                        Math.pow(angularRotationX.getAsDouble(), 3) * drive.getMaximumAngularVelocity(),
                        true,
                        false,
                        getPivot(fl, fr, bl, br));
    });
  }

   /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    drive.driveFieldOriented(velocity);
  }
  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity, Translation2d centerOfRotationMeters)
  {
    drive.driveFieldOriented(velocity, centerOfRotationMeters);
  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner()
  {
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometry,
        this::getRobotVelocity,
        this::setChassisSpeeds,
        new HolonomicPathFollowerConfig(
                                         Constants.AutoConstants.TRANSLATION_PID,
                                         Constants.AutoConstants.ANGLE_PID,
                                         4.5,
                                         // Max module speed, in m/s
                                         drive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                                         // Drive base radius in meters. Distance from robot center to furthest module.
                                         new ReplanningConfig()
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this // Reference to this subsystem to set requirements
                                  );
  }
  public ChassisSpeeds getRobotVelocity()
  {
    return drive.getRobotVelocity();
  }
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    drive.setChassisSpeeds(chassisSpeeds);
  }
  public Pose2d getPose()
  {
    return drive.getPose();
  }
  /**
   * reset odometry to given pose2d
   * @param initialHolonomicPose
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    drive.resetOdometry(initialHolonomicPose);
  }

  /**
   * get center of rotation based on booleanSuppliers for each corner
   * if fl is true, rotation center would be t2d for fl module
   * if multiple boolsupliers are true, output will be average of corrisponding positions
   * @param fl - booleanSupplier for front left  corner
   * @param fr - booleanSupplier for front right corner
   * @param bl - booleanSupplier for back  left  corner
   * @param br - booleanSupplier for back  right corner
   * @return translation2d for rotation center
   */
  public Translation2d getPivot(BooleanSupplier fl, BooleanSupplier fr, BooleanSupplier bl, BooleanSupplier br){

    ArrayList<Translation2d> positions = new ArrayList<>();

    //add module position to list if corisponding button is pressed
    if (fl.getAsBoolean()) positions.add(drive.getModules()[0].getConfiguration().moduleLocation);
    if (fr.getAsBoolean()) positions.add(drive.getModules()[1].getConfiguration().moduleLocation);
    if (bl.getAsBoolean()) positions.add(drive.getModules()[2].getConfiguration().moduleLocation);
    if (br.getAsBoolean()) positions.add(drive.getModules()[3].getConfiguration().moduleLocation);
    //average positions
    return meanAllT2d(positions);
  }

  /** take the mean of all T2ds in arrayList t */
  private Translation2d meanAllT2d(ArrayList<Translation2d> t){
    if(t.size() > 0){
    double[] x = new double[t.size()];
    double[] y = new double[t.size()];

    for (int i = 0; i < t.size(); i++) {x[i] = t.get(i).getX();}
    for (int i = 0; i < t.size(); i++) {y[i] = t.get(i).getY();}

    double meanX = 0;
    for(double a : x){meanX += a;}
    meanX /= x.length;

    double meanY = 0;
    for(double a : y){meanY += a;}
    meanY /= y.length;

    return new Translation2d(meanX, meanY);
    } else {
      return new Translation2d();
    }
  }

  /**
   * Setup the photon vision class.
   */
  public void setupPhotonVision()
  {
    vision = new Vision(drive::getPose, drive.field);
  }

}
