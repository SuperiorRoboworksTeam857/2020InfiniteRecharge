
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.IDs.Controls;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class FrancoisXXI extends TimedRobot {

    // Driver Input
    public Joystick m_joystick = new Joystick(IDs.DriveStation.kJoystick);
    public Joystick m_gamepad = new Joystick(IDs.DriveStation.kGamepad);

    // Drive Motors
    public WPI_TalonFX m_driveFL = new WPI_TalonFX(IDs.CAN.kFrontLeftChannel);
    public WPI_TalonFX m_driveRL = new WPI_TalonFX(IDs.CAN.kRearLeftChannel);
    public WPI_TalonFX m_driveFR = new WPI_TalonFX(IDs.CAN.kFrontRightChannel);
    public WPI_TalonFX m_driveRR = new WPI_TalonFX(IDs.CAN.kRearRightChannel);

    // Game Piece Manipulation
    public WPI_VictorSPX m_intakeMotor = new WPI_VictorSPX(IDs.CAN.kIntake);
    public WPI_VictorSPX m_indexMotor = new WPI_VictorSPX(IDs.CAN.kIndexer);
    public WPI_VictorSPX m_hopperMotor = new WPI_VictorSPX(IDs.CAN.kHopper);

    // - Shooter
    public CANSparkMax m_shooterMotorL = new CANSparkMax(IDs.CAN.kShooterL, MotorType.kBrushless);
    public CANSparkMax m_shooterMotorR = new CANSparkMax(IDs.CAN.kShooterR, MotorType.kBrushless);

    // Climber
    public WPI_TalonSRX m_climberLiftMotor = new WPI_TalonSRX(IDs.CAN.kClimbLift);
    public WPI_TalonSRX m_climberMotorR = new WPI_TalonSRX(IDs.CAN.kClimbR);
    public WPI_TalonSRX m_climberMotorL = new WPI_TalonSRX(IDs.CAN.kClimbL);
    public WPI_VictorSPX m_climberSliderMotor = new WPI_VictorSPX(IDs.CAN.kClimbSlide);

    // Sensors
    public ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

    // - Internal Game Piece Sensors
    // TODO : Hopefully use these at some point?
    public DigitalInput m_indexSensor0 = new DigitalInput(0);
    public DigitalInput m_indexSensor1 = new DigitalInput(1);
    public DigitalInput m_indexSensor2 = new DigitalInput(2);

    // Pneumatics
    public DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(IDs.CAN.kPneumatics, 0, 1);

    // LED Controller
    public Spark m_ledController = new Spark(0);

    // Autonomous
    public Joystick m_autonSelector = new Joystick(IDs.DriveStation.kSwitchboard);
    public Timer m_autonTimer = new Timer();
    public int m_autonMode = 0;
    public int m_autonStage = 0;
    public final double kEncoderTicksPerInch = (2048 * 10.71) / (Math.PI * 6);

    // Field Element Dimensions
    public final double kTargetHeight = 98.25;

    // Robot Dimensions
    public final double kLimelightHeight = 41.0; // TODO: Update when limelight mounted
    public final double kLimelightAngle = -89.0; // TODO: Update when limelight mounted

    // Motor Output Speeds/Limits
    public final double kShooterSpeed = -0.95; // at one tick out speed 0.685l // < Not sure what that meant but I'm keeping it there in case it's important - JD
    public final double kIndexSpeed = -0.9;
    public final double kHopperSpeed = -0.3;
    public final double kIntakeSpeed = 1.0;
    public final double kClimberLiftSpeed = 0.7;
    public final double kClimberSpeed = 1.0;

    public enum MotorSpeed {
        FORWARD, STOPPED, REVERSE;
    }

    @Override
    public void robotInit() {

        // Invert left drive motors
        m_driveFL.setInverted(true);
        m_driveRL.setInverted(true);

        // Reset encoder positions
        resetEncoders();

        // Invert right climber motor
        m_climberMotorR.setInverted(true);

        // Set all drive motors to coast on neutral
        setDriveNeutralMode(NeutralMode.Coast);

        // Calibrate and reset gyro
        m_gyro.calibrate();
        m_gyro.reset();
    }

    @Override
    public void robotPeriodic() {

        // TESTING - Put gyro angle on SmartDashboard
        SmartDashboard.putNumber("gyro angle", m_gyro.getAngle());

        // TESTING - Put encoder values on SmartDashboard
        SmartDashboard.putNumber("L encoder", m_driveFL.getSensorCollection().getIntegratedSensorPosition());
        SmartDashboard.putNumber("R encoder", m_driveFR.getSensorCollection().getIntegratedSensorPosition());
        SmartDashboard.putNumber("L shooter speed", m_shooterMotorL.getEncoder().getVelocity());
        SmartDashboard.putNumber("R shooter speed", m_shooterMotorR.getEncoder().getVelocity());

        // TESTING - Put limelight values on SmartDashboard
        SmartDashboard.putNumber("limelight TX", getLimelightValue("tx")); // Difference of X axis angles between crosshairs
        SmartDashboard.putNumber("limelight TY", getLimelightValue("ty")); // Difference of Y axis angles between crosshairs
        SmartDashboard.putNumber("limelight TA", getLimelightValue("ta")); // Area of target
        SmartDashboard.putNumber("limelight cam", getLimelightValue("camMode")); // Camera mode [0 = VISION | 1 = DRIVER CAM]
        SmartDashboard.putNumber("limelight led", getLimelightValue("ledMode")); // LED Mode [0 = DEFAULT | 1 = FORCE OFF | 2 = FORCE BLINK | 3 = FORCE ON]
        SmartDashboard.putNumber("target distance", getLimelightValue("tDistance")); // Distance from target

        // TESTING - Put drive motor speeds on SmartDashboard
        SmartDashboard.putNumber("FL", m_driveFL.get());
        SmartDashboard.putNumber("FR", m_driveFR.get());
        SmartDashboard.putNumber("RL", m_driveRL.get());
        SmartDashboard.putNumber("RR", m_driveRR.get());
    }

    @Override
    public void autonomousInit() {
        // Reset encoder positions
        resetEncoders();

        // Reset auton timer
        m_autonTimer.start();
        m_autonTimer.reset();

        // Set auton stage to stage 0
        m_autonStage = 0;

        // Set all motor speeds to 0
        stopAll();

        // Reset gyro
        m_gyro.reset();

        // Set intake arm to reversed position
        setIntakeArm(Value.kReverse);

        // Set auton mode to 0 by default
        m_autonMode = 0;

        // Get first switch toggled on drive station to select auton mode
        for (int i = 3; i < 8; i++) {
            if (m_autonSelector.getRawButton(i)) {
                m_autonMode = i - 3;
                break;
            }
        }
    }

    @Override
    public void autonomousPeriodic() {

        // Mode 0: Fire starting payload and back off starting line
        if (m_autonMode == 0) {
            switch (m_autonStage) {
                case (0): // Stage 0: Start ramping up shooter
                    enableShooter(true);

                    if (m_shooterMotorR.getEncoder().getVelocity() <= -4000 || m_autonTimer.get() > 5) {
                        setAutonStage(1);
                    }

                    break;
                case (1): // Stage 1: Fire starting payload
                    enableShooter(true);
                    enableIndexer(true);

                    if (m_autonTimer.get() > 3.2) {
                        setAutonStage(2);
                    }

                    break;
                case (2): // Stage 2: Back up
                    enableIndexer(false);
                    enableShooter(false);

                    if (moveTo(-36, 4)) {
                        setAutonStage(3);
                    }

                    break;
                default: // Default to stop all motors and stop timer
                    stopAll();
                    m_autonTimer.stop();
                    m_autonTimer.reset();
                    break;
            }

        } else if (m_autonMode == 1) {
            // TODO : Make sure this still works I guess?
            switch (m_autonStage) {
                case (0): // Stage 0: Start ramping up shooter
                    enableShooter(true);

                    if (m_shooterMotorR.getEncoder().getVelocity() <= -4000 || m_autonTimer.get() > 5) {
                        setAutonStage(1);
                    }

                    break;
                case (1): // Stage 1: Fire starting payload
                    enableShooter(true);
                    enableIndexer(true);

                    if (m_autonTimer.get() > 3.2) {
                        setAutonStage(2);
                    }

                    break;
                case (2): // Stage 2: Stop shooter and turn to face trench run
                    enableIndexer(false);
                    enableShooter(false);

                    if (turnTo(130, 2)) {
                        setAutonStage(3);
                    }

                    break;
                case (3): // Stage 3: Move to trench run
                    if (moveTo(100, 3)) {
                        setAutonStage(4);
                    }

                    break;
                case (4): // Stage 4: Align to trench run
                    if (turnTo(45, 2)) {
                        setAutonStage(5);
                    }

                    break;
                case (5): // Stage 5: Drop intake
                    stopAll();
                    setIntakeArm(Value.kForward);

                    if (m_autonTimer.get() >= 2) {
                        setAutonStage(6);
                    }

                    break;
                case (6): // Stage 6: Start intake and move forward, collecting powercells
                    setIntake(MotorSpeed.FORWARD);

                    if (moveTo(123, 4)) {
                        setAutonStage(7);
                    }

                    break;
                case (7): // Stage 7: Stop intake and move back to start of trench run
                    setIntake(MotorSpeed.STOPPED);

                    if (moveTo(-123, 4)) {
                        setAutonStage(8);
                    }

                    break;
                default: // Default to stop all motors and stop timer
                    stopAll();
                    m_autonTimer.stop();
                    m_autonTimer.reset();
                    break;
            }
        }
    }

    @Override
    public void teleopInit() {
    }

    // TODO : Update control scheme to account for joystick twist sensitivity when not driving forward/backward
    // TODO : Add speed adjustment control?

    /* 
     * Control Scheme:
     * 
     * Drive: joystick Y forward/backward, twist for rotation
     * Toggle intake arm: joystick fire 6
     * 
     */

    @Override
    public void teleopPeriodic() {

        // Toggle limelight driver camera on joystick fire 5 pressed
        if (m_joystick.getRawButtonPressed(IDs.Controls.kToggleCamButton)) {
            toggleDriverCam();
        }

        // Run shooter on gamepad right trigger pulled
        if (m_gamepad.getRawAxis(IDs.Controls.kDriveShooterAxis) > 0.2) {
            enableShooter(true);
        } else {
            enableShooter(false);
        }

        // Run intake/hopper in on joystick trigger pressed and out on joystick fire 2 pressed (prevent if intake arm is not extended)
        if (m_joystick.getRawButton(IDs.Controls.kRunIntakeNormalButton) && m_intakeSolenoid.get().equals(Value.kForward)) {
            setIntake(MotorSpeed.FORWARD);
        } else if (m_joystick.getRawButton(IDs.Controls.kRunIntakeReverseButton) && m_intakeSolenoid.get().equals(Value.kForward)) {
            setIntake(MotorSpeed.REVERSE);
        } else {
            setIntake(MotorSpeed.STOPPED);
        }

        // Toggle intake arm position on joystick fire 6 pressed
        if (m_joystick.getRawButtonPressed(IDs.Controls.kToggleIntakePositionButton)) {
            if (!m_intakeSolenoid.get().equals(Value.kForward)) {
                setIntakeArm(Value.kForward);
            } else {
                setIntakeArm(Value.kReverse);
            }
        }

        // Enable esophagus on gamepad Y button pressed
        if (m_gamepad.getRawButton(IDs.Controls.kRunEsophagusButton)) {
            enableIndexer(true);
        } else {
            enableIndexer(false);
        }

        // Run hopper if intake or esophagus is running otherwise always run on joystick HAT direction up else stop
        if (((m_joystick.getRawButton(IDs.Controls.kRunIntakeNormalButton) ||
             m_gamepad.getRawButton(IDs.Controls.kRunEsophagusButton)) &&
             m_intakeSolenoid.get().equals(Value.kForward)) || 
             m_joystick.getPOV() == IDs.Controls.kRunHopperPOV){
            enableHopper(true);
        } else {
            enableHopper(false);
        }

        // Drive climber on gamepad directional right pressed, else stop climber and check for climb lift
        if (m_gamepad.getPOV() == Controls.kRunClimbPOV) {

            // Stop climber lift and set to coast
            setClimberLift(0);
            m_climberLiftMotor.setNeutralMode(NeutralMode.Coast);

            // Start climber
            setClimber(kClimberSpeed);

        } else {

            // Stop climber
            setClimber(0);

            // Set climb lift to brake 
            m_climberLiftMotor.setNeutralMode(NeutralMode.Brake);

            // Drive climber lift up on gamepad directional up pressed, down on gamepad directional down pressed
            if (m_gamepad.getPOV() == IDs.Controls.kRunClimbLiftUpPOV) {
                setClimberLift(kClimberLiftSpeed);

            } else if (m_gamepad.getPOV() == IDs.Controls.kRunClimbLiftDownPOV) {
                setClimberLift(-kClimberLiftSpeed);

            } else {
                setClimberLift(0);
            }
        }

        // Drive climb slider with gamepad left joystick X axis
        setClimberSlideMotor(m_gamepad.getRawAxis(IDs.Controls.kDriveClimbSlideAxis));

        // Speed Init
        double speedL = 0.0;
        double speedR = 0.0;

        // Add joystick forward-backward to speed values
        speedL += m_joystick.getRawAxis(IDs.Controls.kDriveFwdAxis);
        speedR += m_joystick.getRawAxis(IDs.Controls.kDriveFwdAxis);

        // Add joystick twist to speed values
        speedL -= m_joystick.getRawAxis(IDs.Controls.kDriveRotationAxis);
        speedR += m_joystick.getRawAxis(IDs.Controls.kDriveRotationAxis);

        // TESTING - Put everything on the SmartDashboard to figure out what the h e c k is going wrong
        SmartDashboard.putNumber("joystick forward", m_joystick.getRawAxis(IDs.Controls.kDriveFwdAxis));
        SmartDashboard.putNumber("joystick twist", m_joystick.getRawAxis(IDs.Controls.kDriveRotationAxis));
        SmartDashboard.putNumber("speedL", speedL);
        SmartDashboard.putNumber("speedL", speedR);

        // Main Drive

        // Aim to vision target on joystick fire 2 pressed, else drive standard
        if (m_joystick.getRawButton(IDs.Controls.kAlignToTargetButton)) {
            // Turn on limelight
            enableLimelight(true);

            // Turn to target and allow distance adjustments from joystick when aligned
            if (turnToTarget()) {
                setDrive(m_joystick.getRawAxis(IDs.Controls.kDriveFwdAxis));
            }

        } else {
            // Turn off limelight
            enableLimelight(false);
            
            // Drive
            setDrive(speedL, speedR);
        }
    }

    @Override
    public void disabledInit() {
        enableLimelight(false);
    }

    public void setAutonStage(int stage) {
        setAutonStage(stage, false);
    }

    public void setAutonStage(int stage, boolean stopAll) {

        resetEncoders();
        m_autonTimer.reset();
        m_gyro.reset();

        if(stopAll) stopAll();

        m_autonStage = stage;
    }

    public void resetEncoders() {
        m_driveFL.getSensorCollection().setIntegratedSensorPosition(0, 0);
        m_driveFR.getSensorCollection().setIntegratedSensorPosition(0, 0);
        m_driveRL.getSensorCollection().setIntegratedSensorPosition(0, 0);
        m_driveRR.getSensorCollection().setIntegratedSensorPosition(0, 0);
    }

    public void stopAll() {
        // Stop all drive motors
        setDrive(0);

        // Stop shooter
        enableShooter(false);

        // Stop intake/indexer/hopper
        setIntake(MotorSpeed.STOPPED);
        enableIndexer(false);
        enableHopper(false);

        // Stop all climber motors
        setClimberLift(0);
        setClimber(0);
        setClimberSlideMotor(0);
    }

    public boolean moveTo(double in, double timeout) {

        final double kP = 0.00001;

        boolean complete = false;

        double encoderAvg = (-m_driveFR.getSensorCollection().getIntegratedSensorPosition()
                + m_driveFL.getSensorCollection().getIntegratedSensorPosition()) / 2;

        double setpoint = in * kEncoderTicksPerInch;
        double err = setpoint - encoderAvg;
        double speed = err * kP;

        SmartDashboard.putNumber("encoderAvg", encoderAvg);
        SmartDashboard.putNumber("err", err);
        SmartDashboard.putNumber("setpoint", setpoint);
        SmartDashboard.putNumber("speed", speed);

        if (Math.abs(err) > 200 && m_autonTimer.get() < timeout) {
            setDrive(-speed);

        } else {
            complete = true;
        }

        return complete;
    }

    public boolean turnTo(double degrees, double timeout) {

        boolean complete = false;

        double gyroAngle = m_gyro.getAngle();

        final double kP = 0.005;
        double setpoint = degrees;
        setpoint = degrees;

        if (setpoint > 180) {
            setpoint = -(360 - setpoint);
        } else if (setpoint < -180) {
            setpoint = 360 + setpoint;
        }

        double err = setpoint - gyroAngle;
        double speed = err * kP;

        SmartDashboard.putNumber("rotate err", err);
        SmartDashboard.putNumber("rotate speed", speed);

        if (Math.abs(err) > 2 && m_autonTimer.get() < timeout) {
            setDriveRotate(-speed);
        } else {
            complete = true;
        }

        return complete;
    }

    public boolean turnToTarget() {

        boolean complete = false;

        double tx = getLimelightValue("tx");

        if (getLimelightValue("camMode") == 1) {
            toggleDriverCam();
        }

        if (Math.abs(tx) > 4.0) {
            setDriveRotate(-(tx / 26) * 0.6);
        } else {
            complete = true;
        }

        return complete;
    }

    public void setDriveRotate(double speed) {
        setDrive(speed, -speed);
    }

    public void setDrive(double speed) {
        setDrive(speed, speed);
    }

    public void setDrive(double speedL, double speedR) {
        m_driveFL.set(speedL);
        m_driveFR.set(speedR);
        m_driveRL.set(speedL);
        m_driveRR.set(speedR);
    }

    public void enableShooter(boolean enabled) {
        if (enabled) {
            m_shooterMotorL.set(kShooterSpeed);
            m_shooterMotorR.set(kShooterSpeed);
        } else {
            m_shooterMotorL.set(0);
            m_shooterMotorR.set(0);
        }
    }

    public void setIntakeArm(Value position) {
        if (position.equals(Value.kForward)) {
            m_intakeSolenoid.set(Value.kForward);
        } else if (position.equals(Value.kReverse)) {
            m_intakeSolenoid.set(Value.kReverse);
        } else {
            m_intakeSolenoid.set(Value.kOff);
        }
    }

    public void setIntake(MotorSpeed direction) {
        if (direction == MotorSpeed.FORWARD) {
            m_intakeMotor.set(kIntakeSpeed);
        } else if (direction == MotorSpeed.REVERSE) {
            m_intakeMotor.set(-kIntakeSpeed);
        } else {
            m_intakeMotor.set(0);
        }
    }

    public void setClimberSlideMotor(double speed) {
        m_climberSliderMotor.set(speed);
    }

    public void enableHopper(boolean enabled) {
        if (enabled) {
            m_hopperMotor.set(kHopperSpeed);
        } else {
            m_hopperMotor.set(0);
        }
    }

    public void enableIndexer(boolean enabled) {
        if (enabled) {
            m_indexMotor.set(kIndexSpeed);
        } else {
            m_indexMotor.set(0);
        }
    }

    public void setClimberLift(double speed) {
        m_climberLiftMotor.set(speed);
    }

    public void setClimber(double speed) {
        m_climberMotorL.set(speed);
        m_climberMotorR.set(speed);
    }

    public void toggleDriverCam() {
        double currentMode = getLimelightValue("camMode");

        if (currentMode == 0) {
            setLimelightValue("camMode", 1);
        } else if (currentMode == 1) {
            setLimelightValue("camMode", 0);
        }
    }

    public void enableLimelight(boolean enabled) {
        if (enabled) {
            setLimelightValue("ledMode", 0);
        } else {
            setLimelightValue("ledMode", 1);
        }
    }

    public double getLimelightValue(String entry) {

        double value = -1.0;

        if (entry.equals("tDistance")) {
            value = (kTargetHeight - kLimelightHeight) / Math.tan(Math.toRadians(kLimelightAngle + getLimelightValue("ty")));
        } else {
            value = NetworkTableInstance.getDefault().getTable("limelight").getEntry(entry).getDouble(-1.0);
        }

        return value;
    }

    public void setLimelightValue(String entry, double value) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry(entry).setDouble(value);
    }

    /**
     * @param mode 0 = default | 1 = rainbow | 2 = solid red | 3 = solid green | 4 = primary color shot
     */
    public void setLEDs(int mode) {
        if (mode == 0) {
            m_ledController.set(0);
        } else if (mode == 1) {
            m_ledController.set(-0.99);
        } else if (mode == 2) {
            m_ledController.set(0.61);
        } else if (mode == 3) {
            m_ledController.set(0.77);
        } else if (mode == 4) {
            m_ledController.set(0.13);
        }
    }

    public void setDriveNeutralMode(NeutralMode mode) {
        m_driveFL.setNeutralMode(mode);
        m_driveFR.setNeutralMode(mode);
        m_driveRL.setNeutralMode(mode);
        m_driveRR.setNeutralMode(mode);
    }

    public double limitSpeed(double d, double limit) {
        if (d > limit) {
            d = limit;
        } else if (d < -limit) {
            d = -limit;
        }
        return d;
    }

}