package frc.utils.rumble;

/**
 * basic controller rumble
 */
public class Rumble implements RumbleBase{
    private double duration = 0.0;
    private double strength = 0.0;
    private RumblePosition pos = RumblePosition.BOTH;

    /**
     * creates a new rumble
     * @param time durration of rumble
     * @param strength power (0-1) of rumble
     */
    public Rumble(double time, double strength){
        this.strength = strength;
        this.duration = time;
    }

    /**
     * creates a new rumble
     * @param time durration of rumble
     * @param strength power (0-1) of rumble
     * @param pos position of rumble(left, both, or right)
     */
    public Rumble(double time, double strength, RumblePosition pos){
        this.strength = strength;
        this.duration = time;
        this.pos = pos;
    }
    @Override
    public void subtractTime(double time) {
        duration -= time;
    }
    @Override
    public double getTime() {
        return duration;
    }
    @Override
    public void setTime(double time) {
        duration = time;
    }
    @Override
    public double getStrength() {
        return strength;
    }
    @Override
    public void update(){
    }
    @Override
    public RumblePosition getPosition() {
        return pos;
    }
}