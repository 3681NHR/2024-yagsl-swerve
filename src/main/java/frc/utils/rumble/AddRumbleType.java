package frc.utils.rumble;

/**
 * how to add time to rumble que
 */
public enum AddRumbleType{
    /** 
     * replaces all rumble data
     */
    OVERRIDE,
    /**
     * adds rumble to que
     */
    ADD,
    /**
     * plays rumble over others
     */
    OVERLAY
}