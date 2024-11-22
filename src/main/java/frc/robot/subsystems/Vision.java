package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import swervelib.SwerveDrive;

public class Vision
{
  private Camera[] cameras = {
    new Camera.builder()
              .withPosition(new Translation3d())
              .withAngle(new Rotation3d())
              .withCamera(new PhotonCamera("front idk"))
              .build(),
    
  };
  private final double maxError = 10;
  private final double maxRotError = 90;

  private AprilTagFieldLayout layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  public Vision(){
    
  }
  public EstimatedRobotPose[] getEstimatedRobotPoses(){
    ArrayList<EstimatedRobotPose> poses = new ArrayList<>();
    for(Camera c : cameras){
      poses.add(c.update());
    }
    return (EstimatedRobotPose[]) poses.toArray();
  }
  public void updatePoseEstimation(SwerveDrive drive){
    for(EstimatedRobotPose e : getEstimatedRobotPoses()){
      if(checkPoseError(e.estimatedPose.toPose2d(), drive.getPose())){
        System.err.println("pose error over "+maxError+" meters or over "+maxRotError+" degrees. WARNING: mesurement is still being added to odometry");
      }
      drive.addVisionMeasurement(e.estimatedPose.toPose2d(), e.timestampSeconds);
    }
  }
  private boolean checkPoseError(Pose2d a, Pose2d b) {
    return a.getTranslation().getDistance(b.getTranslation()) >= maxError || a.getRotation().minus(b.getRotation()).getDegrees() > maxRotError;
  }
    
  
}