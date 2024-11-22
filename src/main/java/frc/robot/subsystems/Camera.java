package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Camera {
    public Rotation3d angle;
    public Translation3d translation;
    public PhotonCamera camera;
    public Camera(){}

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
