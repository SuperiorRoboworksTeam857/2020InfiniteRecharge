/**
 * @author programming@first857.org | first@jacobdixon.us (Jacob Dixon)
 * @version 1.0a
 * @since 2020-01-17
 */

package org.first857.utils;

import edu.wpi.first.wpilibj.Joystick;

/* Xbox One Controller/Logitech F310 Gamepad ID Scheme
 * Notes: With F310 Use XInput ('X' instead of 'D' on back of controller) and Flight mode (MODE LED off).
 * 
 * Button 1:  A
 * Button 2:  B
 * Button 3:  X
 * Button 4:  Y
 * Button 5:  Left Bumper 
 * Button 6:  Right Bumper
 * Button 7:  Share/Back
 * Button 8:  Menu/Start
 * Button 9:  Left Stick Press
 * Button 10: Right Stick Press
 * 
 * Axis 0: Left stick x     Left - Right      (-1.0 - 1.0)
 * Axis 1: Left stick y     Down - Up         (-1.0 - 1.0)
 * Axis 2: Left Trigger     Rest - Pressed    (0.0 - 1.0)
 * Axis 3: Right Trigger    Rest - Pressed    (0.0 - 1.0)
 * Axis 4: Right Stick x    Left - Right      (-1.0 - 1.0)
 * Axis 5: Right Stick y    Down - Up         (-1.0 - 1.0)
 * 
 * POV_ID: Directional Pad 0-8 Segments Standard (Operates clockwise, -1 when not pressed, returns degrees)
 * 
 * Rumble: Is rumble 
 */

/* Saitek ST290 Pro Joytsick ID Scheme
 * Notes: Button labels correct, throttle requires nudge after initialization otherwise inital value could read incorrectly.
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
 * POV_ID: HAT Switch 0-8 Segments Standard (Operates clockwise, -1 when not pressed, returns degrees)
 */

 /* Logitech Attack 3 Joytsick ID Scheme
 * Notes: Button labels correct.
 *
 * Button 1:  Trigger
 * Button 2:  Stick Bottom Center Fire (2)
 * Button 3:  Stick Middle Center Fire (3)
 * Button 4:  Stick Left Fire (4)
 * Button 5:  Stick Right Fire (5)
 * Button 6:  Base Left Front Fire (6)
 * Button 7:  Base Right Rear Fire (7)
 * Button 8:  Base Middle Left (8)
 * Button 9:  Base Middle Right (9)
 * Button 10: Base Left Rear Fire (10)
 * Button 11: Base Right Front Fire (11)
 * 
 * Axis 0: X           Left - Right              (-1.0 - 1.0)
 * Axis 1: Y           Forward - Backward        (-1.0 - 1.0)
 * Axis 2: Throttle    Full-Front - Full-Back    (0.0 - 1.0)
 */

/* Control Types
 *
 * Button:  boolean         true or false 
 * POV:     int (angle)        0 to 360; -1 when not presed
 * Axis:    analog decimal  -1.0 to 1.0
 */ 

public abstract interface Controllers {

    public class Controller extends Joystick {
        public Controller(int port) {
            super(port);
        }
    }

    public class LogitechF310 extends Controller {

        public LogitechF310(int port) {
            super(port);
        }

        public static final int A_BUTTON_ID = 1;
        public static final int B_BUTTON_ID = 2; 
        public static final int X_BUTTON_ID = 3; 
        public static final int Y_BUTTON_ID = 4; 

        public static final int LEFT_BUMPER_BUTTON_ID  = 5; 
        public static final int RIGHT_BUMPER_BUTTON_ID = 6; 
        public static final int BACK_BUTTON_ID         = 7; 
        public static final int START_BUTTON_ID        = 8; 

        public static final int LEFT_STICK_BUTTON_ID  = 9; 
        public static final int RIGHT_STICK_BUTTON_ID = 10; 

        public static final int LEFT_X_AXIS_ID  = 0;
        public static final int LEFT_Y_AXIS_ID  = 1;
        public static final int RIGHT_X_AXIS_ID = 4;
        public static final int RIGHT_Y_AXIS_ID = 5;

        public static final int LEFT_TRIGGER_AXIS_ID  = 2;
        public static final int RIGHT_TRIGGER_AXIS_ID = 3;

        public static final int N_POV_ID  = 0;
        public static final int NE_POV_ID = 45;
        public static final int E_POV_ID  = 90;
        public static final int SE_POV_ID = 135;
        public static final int S_POV_ID  = 180;
        public static final int SW_POV_ID = 225;
        public static final int W_POV_ID  = 270;
        public static final int NW_POV_ID = 315;
    }

    public class XboxOneController extends Controller {

        public XboxOneController(int port) {
            super(port);
        }

        public static final int A_BUTTON_ID = 1;
        public static final int B_BUTTON_ID = 2; 
        public static final int X_BUTTON_ID = 3; 
        public static final int Y_BUTTON_ID = 4; 

        public static final int LEFT_BUMPER_BUTTON_ID  = 5; 
        public static final int RIGHT_BUMPER_BUTTON_ID = 6; 
        public static final int SHARE_BUTTON_ID        = 7; 
        public static final int MENU_BUTTON_ID         = 8; 

        public static final int LEFT_STICK_BUTTON_ID  = 9; 
        public static final int RIGHT_STICK_BUTTON_ID = 10; 

        public static final int LEFT_X_AXIS_ID  = 0;
        public static final int LEFT_Y_AXIS_ID  = 1;
        public static final int RIGHT_X_AXIS_ID = 4;
        public static final int RIGHT_Y_AXIS_ID = 5;

        public static final int LEFT_TRIGGER_AXIS_ID  = 2;
        public static final int RIGHT_TRIGGER_AXIS_ID = 3;

        public static final int N_POV_ID  = 0;
        public static final int NE_POV_ID = 45;
        public static final int E_POV_ID  = 90;
        public static final int SE_POV_ID = 135;
        public static final int S_POV_ID  = 180;
        public static final int SW_POV_ID = 225;
        public static final int W_POV_ID  = 270;
        public static final int NW_POV_ID = 315;
    }

    public class SaitekST290 extends Controller {

        public SaitekST290(int port) {
            super(port);
        }

        public static final int TRIGGER_BUTTON_ID = 1;

        public static final int FIRE_2_BUTTON_ID = 2; // Center
        public static final int FIRE_3_BUTTON_ID = 3; // Left
        public static final int FIRE_4_BUTTON_ID = 4; // Right
        public static final int FIRE_5_BUTTON_ID = 5; // Top Left
        public static final int FIRE_6_BUTTON_ID = 6; // Top Right

        public static final int X_AXIS_ID        = 0;
        public static final int Y_AXIS_ID        = 1;
        public static final int THROTTLE_AXIS_ID = 2;
        public static final int TWIST_AXIS_ID    = 3;

        public static final int N_POV_ID  = 0;
        public static final int NE_POV_ID = 45;
        public static final int E_POV_ID  = 90;
        public static final int SE_POV_ID = 135;
        public static final int S_POV_ID  = 180;
        public static final int SW_POV_ID = 225;
        public static final int W_POV_ID  = 270;
        public static final int NW_POV_ID = 315;
    }

    public class LogitechAttack3 extends Controller {

        public LogitechAttack3(int port) {
            super(port);
        }

        public static final int TRIGGER_BUTTON_ID = 1;

        public static final int FIRE_2_BUTTON_ID  = 2;  // Bottom center
        public static final int FIRE_3_BUTTON_ID  = 3;  // Middle center 
        public static final int FIRE_4_BUTTON_ID  = 4;  // Left
        public static final int FIRE_5_BUTTON_ID  = 5;  // Right

        public static final int FIRE_6_BUTTON_ID  = 6;  // Base left front 
        public static final int FIRE_7_BUTTON_ID  = 7;  // Base left rear
        public static final int FIRE_8_BUTTON_ID  = 8;  // Base middle left
        public static final int FIRE_9_BUTTON_ID  = 9;  // Base middle right
        public static final int FIRE_10_BUTTON_ID = 10; // Base left rear
        public static final int FIRE_11_BUTTON_ID = 11; // Base left front

        public static final int X_AXIS_ID         = 0;
        public static final int Y_AXIS_ID         = 1;
        public static final int THROTTLE_AXIS_ID  = 2;
        
    }

    public class MSP430Switchboard extends Controller {

        public MSP430Switchboard(int port) {
            super(port);
        }

        public static final int SWITCH_0_SWITCH_ID = 3; // Leftmost
        public static final int SWITCH_1_SWITCH_ID = 4;
        public static final int SWITCH_2_SWITCH_ID = 5; // Middle
        public static final int SWITCH_3_SWITCH_ID = 6;
        public static final int SWITCH_4_SWITCH_ID = 7; // Rightmost
    }

    public class LogitechX52 extends Controller {

        public LogitechX52(int port){
            super(port);
        }

        public static final int X_AXIS_ID                  = 0;
        public static final int Y_AXIS_ID                  = 1;
        public static final int THROTTLE_PRIMARY_AXIS_ID   = 2;
        public static final int THROTTLE_SECONDARY_AXIS_ID = 3;
        public static final int THROTTLE_TERTIARY_AXIS_ID  = 4;
        public static final int Z_AXIS_ID                  = 5;
        public static final int THROTTLE_SLIDER_AXIS_ID    = 6;
        

        public static final int PRIMARY_TRIGGER_HALF_BUTTON_ID  = 1;
        public static final int FIRE_BUTTON_ID                  = 2;
        public static final int A_BUTTON_ID                     = 3;
        public static final int B_BUTTON_ID                     = 4;
        public static final int C_BUTTON_ID                     = 5;
        public static final int SECONDARY_TRIGGER_BUTTON_ID     = 6;
        public static final int D_BUTTON_ID                     = 7;
        public static final int E_BUTTON_ID                     = 8;
        public static final int T1_UP_BUTTON_ID                 = 9;
        public static final int T1_DOWN_BUTTON_ID               = 10;
        public static final int T3_UP_BUTTON_ID                 = 11;
        public static final int T3_DOWN_BUTTON_ID               = 12;
        public static final int T5_UP_BUTTON_ID                 = 13;
        public static final int T5_DOWN_BUTTON_ID               = 14;
        public static final int PRIMARY_TRIGGER_FULL_BUTTON_ID  = 15;
        public static final int STICK_SECONDARY_POV_N_BUTTON_ID = 16;

    }
}