package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import org.photonvision.PhotonCamera;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Transform3d;

public class Vision
{
  private Camera[] cameras;

  public Vision(Camera[] cams){
    this.cameras = cams;
  }
  public Transform3d getPosition(Camera c){
    if(c.camera.getLatestResult().hasTargets()){

    }
  }
  private double weightedAverage(Transform3d[] positions, double[] distances){
    
  }
}