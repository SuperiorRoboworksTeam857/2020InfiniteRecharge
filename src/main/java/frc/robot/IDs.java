/**
 * @author programming@first857.org
 * @version 1.0a
 * @since 2020-01-17
 * 
 * Yeah ok so what, i made this way more complex than it needed to be. 
 * It made sense in my head, ok?
 * now everything is s t r u c t u r e d
 */

package frc.robot;

public interface IDs extends JoystickIDs {

    public class Controls {
        // Controls

        // - Driver (Saitek ST290 Pro Joystick)

        // - - Drive
        public static final int kDriveFwdAxis                 = SaitekST290.Y_AXIS;
        public static final int kDriveRotationAxis            = SaitekST290.TWIST_AXIS;
        public static final int kTurboButton                  = SaitekST290.FIRE_4_BUTTON;

        // - - Systems

        // - - - Intake
        public static final int kRunIntakeNormalButton         = SaitekST290.TRIGGER_BUTTON;
        public static final int kRunIntakeReverseButton        = SaitekST290.FIRE_2_BUTTON;
        public static final int kToggleIntakePositionButton    = SaitekST290.FIRE_6_BUTTON;

        // - - - Limelight 
        public static final int kToggleCamButton               = SaitekST290.FIRE_3_BUTTON;
        public static final int kAlignToTargetButton           = SaitekST290.FIRE_5_BUTTON;

        // - - - Hopper
        public static final int kRunHopperPOV                  = SaitekST290.S_POV;

        // Operator (Logitech F310)

        // - Shooter
        public static final int kDriveShooterAxis              = LogitechF310.RIGHT_TRIGGER_AXIS;

        // - Climber
        public static final int kRunClimbLiftUpPOV             = LogitechF310.N_POV;
        public static final int kRunClimbLiftDownPOV           = LogitechF310.S_POV;
        public static final int kRunClimbPOV                   = LogitechF310.E_POV;
        public static final int kDriveClimbSlideAxis           = LogitechF310.RIGHT_X_AXIS;

        // - Esophagus
        public static final int kRunEsophagusButton            = LogitechF310.Y_BUTTON;
    }

    public class DriveStation {
        // Drive Station

        // - USB Input Devices
        public static final int kJoystick    = 0;
        public static final int kGamepad     = 1;
        public static final int kSwitchboard = 2;
    }

    public class CAN {
        // Robot CAN

        // - Drive (TalonFX)
        public static final int kRearLeftChannel   = 30;
        public static final int kFrontLeftChannel  = 31;
        public static final int kFrontRightChannel = 32;
        public static final int kRearRightChannel  = 33;

        // - Game Piece Manipulation (Victor SPX)
        public static final int kIntake  = 34;
        public static final int kClimb   = 35;
        public static final int kHopper  = 36;
        public static final int kIndexer = 37;

        // - Climber (Victor SPX)
        public static final int kClimbL     = 25;
        public static final int kClimbR     = 23;
        public static final int kClimbLift  = 24;
        public static final int kClimbSlide = 35;

        // - Shooter (Spark Max)
        public static final int kShooterL = 39;
        public static final int kShooterR = 38;

        // - Woosh
        public static final int kPneumatics = 2;
    }
}
