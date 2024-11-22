package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Camera {
    private Rotation3d angle;
    private Translation3d translation;
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;

    private AprilTagFieldLayout layout;
    public Camera(AprilTagFieldLayout layout){
        this.layout = layout;
    }

    public Camera withAngle(Rotation3d rot){
        this.angle = rot;
        return this;
    }
    public Camera withPosition(Translation3d pos){
        this.translation = pos;
        return this;
    }
    public Camera withCamera(PhotonCamera cam){
        this.camera = cam;
        return this;
    }

    public boolean hasTarget(){
        return camera.getLatestResult().hasTargets();
    }
    public PhotonTrackedTarget getBestTarget(){
        PhotonPipelineResult res = camera.getLatestResult();
        if(res.hasTargets()){
            return res.getBestTarget();
        }
        return null;
    }
    public PhotonTrackedTarget[] getAllTargets(){
        PhotonPipelineResult res = camera.getLatestResult();
        if(res.hasTargets()){
            return (PhotonTrackedTarget[]) res.getTargets().toArray();
        }
        return null;
    }
}
