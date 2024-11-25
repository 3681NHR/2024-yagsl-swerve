package frc.utils.rumble;

public interface RumbleBase{
    /**
     * removes time from rumble
     * @param time how much to remove. if greater than remaining time, time will become 0
     */
    public void subtractTime(double time);
    /**
     * @return current duration remaining
     */
    public double getTime();
    /**
     * seet duration of rumble
     * @param time duration
     */
    public void setTime(double time);
    /**
     * @return current strength of rumble(0-1)
     */
    public double getStrength();
    /**
     * update rumble, used on dynamic rumbles like sequences
     */
    public void update();
    /**
     * @return position of rumble
     */
    public RumblePosition getPosition();
}