/**
 * @author programming@first857.org
 * @version 1.0a
 * @since 2020-01-17
 */

package frc.robot;

/* Xbox One Controller/Logitech F310 Gamepad ID Scheme
 * Notes: With F310 Use XInput ('X' instead of 'D' on back of controller) and Flight mode (MODE LED off).
 * 
 * Button 1: A
 * Button 2: B
 * Button 3: X
 * Button 4: Y
 * Button 5: Left Bumper 
 * Button 6: Right Bumper
 * Button 7: Share/Back
 * Button 8: Menu/Start
 * Button 9: Left Stick Press
 * Button 10: Right Stick Press
 * 
 * Axis 0: Left stick x     Left - Right      (-1.0 - 1.0)
 * Axis 1: Left stick y     Down - Up         (-1.0 - 1.0)
 * Axis 2: Left Trigger     Rest - Pressed    (0.0 - 1.0)
 * Axis 3: Right Trigger    Rest - Pressed    (0.0 - 1.0)
 * Axis 4: Right Stick x    Left - Right      (-1.0 - 1.0)
 * Axis 5: Right Stick y    Down - Up         (-1.0 - 1.0)
 * 
 * POV: Directional Pad 0-8 Segments Standard (Operates clockwise, -1 when not pressed, returns degrees)
 * 
 * Rumble: Is rumble 
 */

/* Saitek ST290 Pro Joytsick ID Scheme
 * Notes: Button labels correct, throttle requires nudge after initialization otherwise value could read incorrectly.
 *
 * Button 1: Trigger
 * Button 2: Fire Center (2)
 * Button 3: Fire Left (3)
 * Button 4: Fire Right (4)
 * Button 5: Bumper Fire Left (5) 
 * Button 6: Bumper Fire Right (6)
 * 
 * Axis 0: X           Left - Right              (-1.0 - 1.0)
 * Axis 1: Y           Forward - Backward        (-1.0 - 1.0)
 * Axis 2: Throttle    Full-Front - Full-Back    (0.0 - 1.0)
 * Axis 3: Twist       Left - Right              (-1.0 - 1.0)
 * 
 * POV: HAT Switch 0-8 Segments Standard (Operates clockwise, -1 when not pressed, returns degrees)
 */

public class IDs {

    public class ControllerIDs{
        public static final int A_BUTTON = 1; 
        public static final int B_BUTTON = 2; 
        public static final int X_BUTTON = 3; 
        public static final int Y_BUTTON = 4; 

        public static final int LEFT_BUMPER_BUTTON  = 5; 
        public static final int RIGHT_BUMPER_BUTTON = 6; 
        public static final int BACK_BUTTON         = 7; 
        public static final int START_BUTTON        = 8; 

        public static final int LEFT_STICK_BUTTON  = 9; 
        public static final int RIGHT_STICK_BUTTON = 10; 

        public static final int LEFT_X_AXIS  = 0;
        public static final int LEFT_Y_AXIS  = 1;
        public static final int RIGHT_X_AXIS = 4;
        public static final int RIGHT_Y_AXIS = 5;

        public static final int LEFT_TRIGGER_AXIS  = 2;
        public static final int RIGHT_TRIGGER_AXIS = 3;
    }

    public class JoystickIDs{

        public static final int TRIGGER_BUTTON = 1;

        public static final int FIRE_2 = 2; // Center
        public static final int FIRE_3 = 3; // Left
        public static final int FIRE_4 = 4; // Right
        public static final int FIRE_5 = 5; // Top Left
        public static final int FIRE_6 = 6; // Top Right

        public static final int X_AXIS        = 0;
        public static final int Y_AXIS        = 1;
        public static final int THROTTLE_AXIS = 2;
        public static final int TWIST_AXIS   = 3;
    }

    // USB Input Devices
    public static final int kJoystick = 0;
    public static final int kGamepad  = 1;
    
    // Drive (TalonFX)
    public static final int kRearLeftChannel   = 30;
    public static final int kFrontLeftChannel  = 31;
    public static final int kFrontRightChannel = 32;
    public static final int kRearRightChannel  = 33;

    // Game Piece Manipulation (Victor SPX)
    public static final int kIntake  = 34;
    public static final int kClimb   = 35;
    public static final int kHopper  = 36;
    public static final int kIndexer = 37;

    // Shooters (Spark Max)
    public static final int kShooterL = 39;
    public static final int kShooterR = 38;

    // Woosh
    public static final int kPneumatics = 2;
}
