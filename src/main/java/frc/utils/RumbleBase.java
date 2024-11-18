package frc.utils;

public interface RumbleBase{
    public void subtractTime(double time);
    public double getTime();
    public void setTime(double time);
    public double getStrength();
    public void update();
}