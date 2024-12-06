package frc.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * handles timing and dashboard timers
 */
public class TimerHandler {

    private static double autoStart = 0.0;
    private static double teleopStart = 0.0;

    public static void updateTeleop(){
        SmartDashboard.putNumber("time/teleop/UpTime"       , Timer.getFPGATimestamp()-teleopStart);
        SmartDashboard.putNumber("time/teleop/RemainingTime", MathUtil.clamp(Constants.TELEOP_TIME-(Timer.getFPGATimestamp()-teleopStart), 0, 10000));
        SmartDashboard.putNumber("time/RemainingTime", MathUtil.clamp(Constants.TELEOP_TIME-(Timer.getFPGATimestamp()-teleopStart), 0, 100000));
    }
    public static void updateAuto(){
        SmartDashboard.putNumber("time/auto/UpTime"       , Timer.getFPGATimestamp()-autoStart);
        SmartDashboard.putNumber("time/auto/RemainingTime", MathUtil.clamp(Constants.AUTO_TIME-(Timer.getFPGATimestamp()-autoStart), 0, 100000));
        SmartDashboard.putNumber("time/RemainingTime", MathUtil.clamp(Constants.AUTO_TIME-(Timer.getFPGATimestamp()-autoStart), 0, 100000));
    }
    public static void initTeleop(){
        teleopStart = Timer.getFPGATimestamp();
        SmartDashboard.putNumber("time/teleop/StartTime", teleopStart);
    }
    public static void initAuto(){
        autoStart = Timer.getFPGATimestamp();
        SmartDashboard.putNumber("time/auto/StartTime", autoStart);
    }
    public static void init(){
        SmartDashboard.putNumber("time/upTime", 0.0);
        SmartDashboard.putNumber("time/RemainingTime", 0);

        SmartDashboard.putNumber("time/auto/StartTime"    , 0);
        SmartDashboard.putNumber("time/auto/UpTime"       , 0);
        SmartDashboard.putNumber("time/auto/RemainingTime", Constants.AUTO_TIME);
        SmartDashboard.putNumber("time/auto/TotalTime"    , Constants.AUTO_TIME);
        
        SmartDashboard.putNumber("time/teleop/StartTime"    , 0);
        SmartDashboard.putNumber("time/teleop/UpTime"       , 0);
        SmartDashboard.putNumber("time/teleop/RemainingTime", Constants.TELEOP_TIME);
        SmartDashboard.putNumber("time/teleop/TotalTime"    , Constants.TELEOP_TIME);
    }
    public static void update(){
        SmartDashboard.putNumber("time/upTime", Timer.getFPGATimestamp());
    } 
    public static double getTeleopRemaining(){
        if(teleopStart > 0){
            return MathUtil.clamp(Constants.TELEOP_TIME-(Timer.getFPGATimestamp()-teleopStart), 0, 100000);
        } else {
            return Constants.TELEOP_TIME;
        }
    }
    public static double getAutoRemaining(){
        if(autoStart > 0){
            return MathUtil.clamp(Constants.AUTO_TIME-(Timer.getFPGATimestamp()-autoStart), 0, 100000);
        } else {
            return Constants.AUTO_TIME;
        }
    }
    
}
