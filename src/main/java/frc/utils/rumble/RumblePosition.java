package frc.utils.rumble;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/**
 * which rumble motor to use(left, righ, both)
 */
public enum RumblePosition {
    LEFT,
    RIGHT,
    BOTH;

    /**
     * @return rumblePosition as GenericHID.RumbleType, used when setting rumble
     */
    public RumbleType asNativeEnum(){
        switch (this) {
            case LEFT:
            return RumbleType.kLeftRumble;
            case RIGHT:
            return RumbleType.kRightRumble;
            case BOTH:
            return RumbleType.kBothRumble;
            
            default:
                return null;
        }
    }
}
