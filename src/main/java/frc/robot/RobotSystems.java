package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public interface RobotSystems {

    public final double kDriveSpeedMultipliter = 1.0;
    public final double kDriveSpeedMultipliterTurbo = 1.0;
    public final double kMaxDriveOutput = 1.0;
    public final double kShooterSpeed = -0.95;
    public final double kIndexSpeed = -0.9;
    public final double kHopperSpeed = -0.3;
    public final double kIntakeSpeed = 1.0;
    public final double kClimberLiftSpeed = 0.7;
    public final double kClimberSpeed = 1.0;

    public enum MotorSpeed {
        FORWARD, STOPPED, REVERSE;
    }

    public class Esophagus {

        public static WPI_VictorSPX motor;

        public static boolean running = false;

        public static void run() {
            motor.set(kIndexSpeed);
            running = true;
        }

        public static void stop() {
            motor.set(0);
            running = false;
        }

        public static WPI_VictorSPX getMotor(){
            return motor;
        }

        public static boolean isRunning() {
            return running;
        }
    }

    public class Intake {

        private static DoubleSolenoid solenoid;
        private static WPI_VictorSPX motor;

        private static boolean running = false;
        private static MotorSpeed state = MotorSpeed.STOPPED;

        private static Value position = Value.kOff;

        public static void extend() {
            position = Value.kForward;
            solenoid.set(Value.kForward);
        }

        public static void retract() {
            position = Value.kReverse;
            solenoid.set(Value.kReverse);
        }

        public static void setPosition(Value position) {
            solenoid.set(position);
        }

        public static DoubleSolenoid getSolenoid(){
            return solenoid;
        }

        public static WPI_VictorSPX getMotor(){
            return motor;
        }
        
        public static void run() {
            run(MotorSpeed.FORWARD);
        }

        public static void run(MotorSpeed direction) {
            motor.set(kIntakeSpeed);
            running = true;
            state = direction;
        }

        public static void stop(){
            running = false;
            state = MotorSpeed.STOPPED;
            motor.set(0);
        }
    }

    public class Shooter {

        private boolean running = false;

        public void run(){
            running = true;
        }

        public void stop(){
            running = false;
        }
    }

    public class Hopper {
        
        private boolean running = false;

        public void run(){
            running = true;
        }

        public void stop(){
            running = false;
        }
    }

    public class Climber {

        private boolean running    = false;
        private MotorSpeed state   = MotorSpeed.STOPPED;

        private boolean extending  = false;
        private boolean climbing   = false;
        private boolean adjudting  = false;

        

        public void stop(){
            running    = false;
            state      = MotorSpeed.STOPPED;
            extending  = false;
            climbing   = false;
            adjudting  = false;
        }
    }
}