package frc.utils.rumble;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * handler for rumble on xbox controller
 */
public class ControllerRumble {
    private XboxController controller;
    List<RumbleBase> rumbles = new ArrayList<>();
    public ControllerRumble(XboxController controller){
        this.controller = controller;
    }
    /**
     * adds a new rumble to the rumble que
     * @param time length of rumble
     * @param power power of rumble
     * @param type how to add to que
     */
    public void addRumble(double time, double power, AddRumbleType type){
        switch (type) {
            case OVERRIDE:
                rumbles.clear();
                rumbles.add(new Rumble(time, power));    
            break;
            case ADD:
                rumbles.add(new Rumble(time, power));
            break;
            case OVERLAY:
            if(rumbles.size() > 0){
                RumbleBase r = rumbles.get(0);
                double t = time;
                while(t>0) {
                    if(t <= r.getTime()){
                        r.subtractTime(t);
                        t = 0;
                    } else {
                        double temp = r.getTime();
                        r.setTime(0);
                        t -= temp;
                    }
                    if(rumbles.get(0).getTime() <= 0){
                        rumbles.remove(0);
                    }
                    
                    if(rumbles.size() > 0){
                        r = rumbles.get(0);
                    } else {
                        break;
                    }
                }
            }
            rumbles.add(0, new Rumble(time, power));
            break;
        
            default:
                break;
        }
    }
    
    /**
     * add rumble sequence to rumble que
     * @param rumble sequence to add
     * @param type how to add to que
     */
    public void addRumble(RumbleSequence rumble, AddRumbleType type){
        switch (type) {
            case OVERRIDE:
                rumbles.clear();
                rumbles.add(rumble);    
            break;
            case ADD:
                rumbles.add(rumble);
            break;
            case OVERLAY:
            if(rumbles.size() > 0){
                RumbleBase r = rumbles.get(0);
                double t = r.getTime();
                while(t>0) {
                    if(t <= r.getTime()){
                        r.subtractTime(t);
                        t = r.getTime();
                    } else {
                        double temp = r.getTime();
                        r.setTime(0);
                        t -= temp;
                    }
                    if(rumbles.size() > 0){
                        if(rumbles.get(0).getTime() <= 0){
                            rumbles.remove(0);
                        }
                    }
                    if(rumbles.size() > 0){
                        r = rumbles.get(0);
                    } else {
                        break;
                    }
                }
            }
            rumbles.add(0, rumble);
        
            default:
                break;
        }
    }
    
    /**
     * clears the rumble que and stops controller rumble
     */
    public void clearRumble(){
        rumbles.clear();
        controller.setRumble(RumbleType.kBothRumble, 0);
    }

    /**
     * advance que by 0.02 seconds and performes timelessupdate
     */
    public void update(){
        if(rumbles.size() > 0){
            rumbles.get(0).subtractTime(0.02);
            rumbles.get(0).update();
        }
        timelessUpdate();
        
    }
    /**
     * main update. does not change time values.
     * removes any completed rumbles and ends rumble if there are no rumbles in que
     * also updates controller rumble state
     */
    public void timelessUpdate(){
        if(rumbles.size() > 0){
            if(rumbles.get(0).getTime() <= 0){
                rumbles.remove(0);
            }
        }
        if(rumbles.size() > 0){
            controller.setRumble(rumbles.get(0).getPosition().asNativeEnum(), rumbles.get(0).getStrength());
        } else {
            controller.setRumble(RumbleType.kBothRumble, 0);
        }
    }
}
