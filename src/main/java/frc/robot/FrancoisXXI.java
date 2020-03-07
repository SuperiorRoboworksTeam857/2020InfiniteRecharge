/**
 * @author programming@first857.org | first@jacobdixon.us (Jacob Dixon)
 * @version 1.0a
 * @since 2020-01-17
 */

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

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.IDs;
import frc.robot.IDs.Controls;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.first857.utils.Maths;
import org.first857.utils.Safety;
import org.first857.utils.Controllers.LogitechF310;
import org.first857.utils.Controllers.SaitekST290;
import org.first857.utils.Controllers.MSP430Switchboard;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class FrancoisXXI extends TimedRobot {

    // Driver Input
    public static SaitekST290 m_joystick = new SaitekST290(IDs.DriveStation.kJoystick);
    public static LogitechF310 m_gamepad = new LogitechF310(IDs.DriveStation.kGamepad);
    public static MSP430Switchboard m_switchboard = new MSP430Switchboard(IDs.DriveStation.kSwitchboard);

    // Drive Motors
    public static WPI_TalonFX m_driveFL = new WPI_TalonFX(IDs.CAN.kFrontLeftChannel);
    // public static WPI_TalonFX m_driveRL = new WPI_TalonFX(IDs.CAN.kRearLeftChannel);
    public static WPI_TalonFX m_driveFR = new WPI_TalonFX(IDs.CAN.kFrontRightChannel);
    // public static WPI_TalonFX m_driveRR = new WPI_TalonFX(IDs.CAN.kRearRightChannel);

    // Drive Groups
    public static SpeedControllerGroup m_driveL = new SpeedControllerGroup(m_driveFL);
    public static SpeedControllerGroup m_driveR = new SpeedControllerGroup(m_driveFR);

    // Drive
    public static DifferentialDrive m_drive = new DifferentialDrive(m_driveL, m_driveR);

    // Game Piece Manipulation
    public static WPI_VictorSPX m_intakeMotor = new WPI_VictorSPX(IDs.CAN.kIntake);
    public static WPI_VictorSPX m_esophagusMotor = new WPI_VictorSPX(IDs.CAN.kEsophagus);
    public static WPI_VictorSPX m_hopperMotor = new WPI_VictorSPX(IDs.CAN.kHopper);

    // - Shooter
    public static CANSparkMax m_shooterMotorL = new CANSparkMax(IDs.CAN.kShooterL, MotorType.kBrushless);
    public static CANSparkMax m_shooterMotorR = new CANSparkMax(IDs.CAN.kShooterR, MotorType.kBrushless);

    // Climber
    public static WPI_TalonSRX m_climberLiftMotor = new WPI_TalonSRX(IDs.CAN.kClimbLift);
    public static WPI_TalonSRX m_climberMotorR = new WPI_TalonSRX(IDs.CAN.kClimbR);
    public static WPI_TalonSRX m_climberMotorL = new WPI_TalonSRX(IDs.CAN.kClimbL);
    public static WPI_VictorSPX m_climberSliderMotor = new WPI_VictorSPX(IDs.CAN.kClimbSlide);

    // Sensors
    public static ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

    // - Internal Game Piece Sensors
    // TODO : Hopefully use these at some point?
    public static DigitalInput m_esophagusSensorBottom = new DigitalInput(0);
    public static DigitalInput m_esophagusSensorTop = new DigitalInput(1);

    // Pneumatics
    public static DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(IDs.CAN.kPneumatics, 0, 1);

    // LED Controller
    // public static Spark m_ledController = new Spark(0);
    public static LEDmode ledState = LEDmode.DEFAULT;

    // Autonomous
    public static Timer m_autonTimer = new Timer();
    public static int m_autonMode = 0;
    public static int m_autonStage = 0;
    public static double m_autonPIDlastTimestamp = 0.0;
    public static double m_autonPIDerrSum = 0.0;

    public static final double kEncoderTicksPerInch = (2048 * 10.71) / (Math.PI * 6);

    // Field Element Dimensions
    public static final double kTargetHeight = 98.25;

    // Robot Dimensions
    public static final double kLimelightHeight = 41.0; // TODO: Update when limelight mounted
    public static final double kLimelightAngle = -89.0; // TODO: Update when limelight mounted

    // Motor Output Speeds/Limits
    public static final double kShooterSpeed = -1.0; // at one tick out speed 0.685l // < Not sure what that meant but I'm keeping it there in case it's important - JD
    public static final double kIndexSpeed = -0.9;
    public static final double kHopperSpeed = -0.3;
    public static final double kIntakeSpeed = 1.0;
    public static final double kClimberLiftSpeed = 0.7;
    public static final double kClimberSpeed = 1.0;

    public static enum MotorSpeed {
        FORWARD, STOPPED, REVERSE;
    }

    public static enum LEDmode {
        DEFAULT, RGB, RED, ORANGE, GREEN,  SHOT_A, SHOT_B;
    }

    @Override
    public void robotInit() {

        // Invert left drive motors
        m_driveFL.setInverted(true);
        // m_driveRL.setInverted(true);

        // Start automatic camera capture
        CameraServer.getInstance().startAutomaticCapture();

        // Reset encoder positions
        resetEncoders();

        // Invert right climber motor
        m_climberMotorR.setInverted(true);

        // Set all drive motors to coast on neutral
        setDriveNeutralMode(NeutralMode.Coast);

        // Calibrate and reset gyro
        m_gyro.calibrate();
        m_gyro.reset();

        // Turn off limelight
        enableLimelight(false);

        // Set LED mode
        setLEDs(LEDmode.RGB);
    }

    @Override
    public void robotPeriodic() {

        // TESTING - Put gyro angle on SmartDashboard
        SmartDashboard.putNumber("gyro angle", m_gyro.getAngle());

        // TESTING - Put drive motor speeds on SmartDashboard
        SmartDashboard.putNumber("FL", m_driveFL.getSensorCollection().getIntegratedSensorVelocity());
        SmartDashboard.putNumber("FR", m_driveFR.getSensorCollection().getIntegratedSensorVelocity());

        SmartDashboard.putBoolean("esophagus 0", m_esophagusSensorBottom.get());
        SmartDashboard.putBoolean("esophagus 1", m_esophagusSensorTop.get());

        SmartDashboard.putNumber("auto mode", m_autonMode);

        for (int i = 3; i < 8; i++) {
            if (m_switchboard.getRawButton(i)) {
                m_autonMode = i - 3;
                break;
            }
        }
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

        // Set PID timestamp and reset PID error sum
        m_autonPIDlastTimestamp = Timer.getFPGATimestamp();
        m_autonPIDerrSum = 0.0;

        // Get first switch toggled on drive station to select auton mode
        for (int i = 3; i < 8; i++) {
            if (m_switchboard.getRawButton(i)) {
                m_autonMode = i - 3;
                break;
            }
        }
    }

    @Override
    public void autonomousPeriodic() {

        if (m_autonMode == 0) { // Mode 0: Fire starting payload and back off starting line 3ft

            switch (m_autonStage) {
                case (0): // Stage 0: Start ramping up shooter
                    enableShooter(true);

                    if (isShooterAtSpeed() || m_autonTimer.get() > 5) {
                        setAutonStage(1);
                    }

                    break;
                case (1): // Stage 1: Fire starting payload
                    enableShooter(true);
                    enableEsophagus(true);

                    if (m_autonTimer.get() > 3.2) {
                        setAutonStage(2);
                    }

                    break;
                case (2): // Stage 2: Back up
                    enableEsophagus(false);
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

        } else if (m_autonMode == 1) { // Mode 1: Fire starting payload and move forwards off starting line 3ft

            switch (m_autonStage) {
                case (0): // Stage 0: Start ramping up shooter
                    enableShooter(true);

                    if (isShooterAtSpeed() || m_autonTimer.get() > 5) {
                        setAutonStage(1);
                    }

                    break;
                case (1): // Stage 1: Fire starting payload
                    enableShooter(true);
                    enableEsophagus(true);

                    if (m_autonTimer.get() > 3.2) {
                        setAutonStage(2);
                    }

                    break;
                case (2): // Stage 2: Move forward
                    enableEsophagus(false);
                    enableShooter(false);

                    if (moveTo(36, 4)) {
                        setAutonStage(3);
                    }

                    break;
                default: // Default to stop all motors and stop timer
                    stopAll();
                    m_autonTimer.stop();
                    m_autonTimer.reset();
                    break;
            }

        } else if (m_autonMode == 2) { // Mode 2: Fire starting payload and back off starting line 6ft

            switch (m_autonStage) {
                case (0): // Stage 0: Start ramping up shooter
                    enableShooter(true);

                    if (isShooterAtSpeed() || m_autonTimer.get() > 5) {
                        setAutonStage(1);
                    }

                    break;
                case (1): // Stage 1: Fire starting payload
                    enableShooter(true);
                    enableEsophagus(true);

                    if (m_autonTimer.get() > 3.2) {
                        setAutonStage(2);
                    }

                    break;
                case (2): // Stage 2: Move forward
                    enableEsophagus(false);
                    enableShooter(false);

                    if (moveTo(-72, 4)) {
                        setAutonStage(3);
                    }

                    break;
                default: // Default to stop all motors and stop timer
                    stopAll();
                    m_autonTimer.stop();
                    m_autonTimer.reset();
                    break;
            }

        } else if (m_autonMode == 3) { // Mode 3: Fire starting payload then relaod at trench
            
            // TODO : Make sure this still works I guess?
            switch (m_autonStage) {
                case (0): // Stage 0: Start ramping up shooter
                    enableShooter(true);

                    if (isShooterAtSpeed() || m_autonTimer.get() > 5) {
                        setAutonStage(1);
                    }

                    break;
                case (1): // Stage 1: Fire starting payload
                    enableShooter(true);
                    enableEsophagus(true);

                    if (m_autonTimer.get() > 3.2) {
                        setAutonStage(2);
                    }

                    break;
                case (2): // Stage 2: Stop shooter and turn to face trench run
                    enableEsophagus(false);
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
        } else if (m_autonMode == 4) { // Mode 4: Fire starting payload with auto-aim then reload at trench
            
            switch (m_autonStage) {
                case (0): // Stage 0
                    enableShooter(true);
                    if(turnToTarget() && isShooterAtSpeed()){
                        setAutonStage(1);
                    }

                    break;
                case (1): // Stage 1
                    setDrive(0);
                    enableEsophagus(true);
                    enableShooter(true);
                    if (m_autonTimer.get() > 3.2) {
                        setAutonStage(2);
                    }

                    break;
                case (2): // Stage 2
                    enableShooter(false);
                    enableEsophagus(false);
                    if(turnTo(0, 2)){
                        setAutonStage(3);
                    }

                    break;
                case (3): // Stage 3
                    if(turnTo(130, 2)){
                        setAutonStage(3);
                    }

                    break;
                case (4): // Stage 4
                    if(moveTo(100, 3)){
                        setAutonStage(4);
                    }
                    
                    break;
                case (5): // Stage 5
                    if (turnTo(45, 2)) {
                        setAutonStage(5);
                    }

                    break;
                case (6): // Stage 6
                    stopAll();
                    setIntakeArm(Value.kForward);
                    if (m_autonTimer.get() >= 2) {
                        setAutonStage(6);
                    }

                    break;
                case (7): // Stage 7
                    setIntake(MotorSpeed.FORWARD);
                    if (moveTo(123, 4)) {
                        setAutonStage(7);
                    }

                    break;
                case (8): // Stage 8
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

    @Override
    public void teleopPeriodic() {

        LEDmode nextLEDstate = LEDmode.DEFAULT;

        // Toggle limelight driver camera on button press
        if (m_joystick.getRawButtonPressed(IDs.Controls.kToggleCamButton)) {
            toggleDriverCam();
        }

        // Run shooter on trigger
        if (m_gamepad.getRawAxis(IDs.Controls.kDriveShooterAxis) > 0.2) {
            enableShooter(true);
            if (isShooterAtSpeed()){
                nextLEDstate = LEDmode.GREEN;
            } else {
                nextLEDstate = LEDmode.RED;
            }
        } else {
            enableShooter(false);
        }

        // Run intake in on button press and out on button press (prevent if intake arm is not extended)
        if (m_joystick.getRawButton(IDs.Controls.kRunIntakeNormalButton) && !m_intakeSolenoid.get().equals(Value.kReverse)) {
            setIntake(MotorSpeed.FORWARD);
        } else if (m_joystick.getRawButton(IDs.Controls.kRunIntakeReverseButton) && !m_intakeSolenoid.get().equals(Value.kReverse)) {
            setIntake(MotorSpeed.REVERSE);
        } else {
            setIntake(MotorSpeed.STOPPED);
        }

        // Toggle intake arm position on button press
        if (m_joystick.getRawButtonPressed(IDs.Controls.kToggleIntakePositionButton)) {
            if (!m_intakeSolenoid.get().equals(Value.kForward)) {
                setIntakeArm(Value.kForward);
            } else {
                setIntakeArm(Value.kReverse);
            }
        }

        // Run esophagus on button press, else auto index
        boolean runEsophagusAuto = false;

        if((m_gamepad.getRawAxis(IDs.Controls.kRunEsophagusAxis) > 0.2 && isShooterAtSpeed()) || 
            m_gamepad.getRawButton(IDs.Controls.kRunEsophagusOverrideButton)){
            enableEsophagus(true);
        } else {
            // Run esophagus when powercell is at esophagus entry and not at shooter
            runEsophagusAuto = enableEsophagusAuto(m_gamepad.getRawButton(IDs.Controls.kRunEsophagusAxis));
        }

        // Run hopper if intake or esophagus is running otherwise always run on button press
        if (m_joystick.getPOV() == (IDs.Controls.kRunHopperPOV)){
            enableHopper(true);
        } else if (m_joystick.getRawButton(IDs.Controls.kRunIntakeNormalButton) ||
                   m_joystick.getRawButton(IDs.Controls.kRunIntakeReverseButton) || 
                   m_gamepad.getRawButton(IDs.Controls.kRunEsophagusAxis) || 
                   runEsophagusAuto) {
            enableHopper(true);
        } else {
            enableHopper(false);
        }
        // Drive climber on button press
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

            // Drive climber lift up on button press and down on button press
            if (m_gamepad.getPOV() == IDs.Controls.kRunClimbLiftUpPOV) {
                setClimberLift(kClimberLiftSpeed);

            } else if (m_gamepad.getPOV() == IDs.Controls.kRunClimbLiftDownPOV) {
                setClimberLift(-kClimberLiftSpeed);

            } else {
                setClimberLift(0);
            }
        }

        // Drive climb slider on trigger
        setClimberSlideMotor(Maths.deadband(m_gamepad.getRawAxis(IDs.Controls.kDriveClimbSlideAxis), 0.1));

        // Main Drive
        double speedForward  = m_joystick.getRawAxis(IDs.Controls.kDriveForwardAxis);
        double speedRotation = m_joystick.getRawAxis(IDs.Controls.kDriveRotateAxis);

        // Slow rotate speed if throttle is below 0.5
        if(m_joystick.getRawAxis(IDs.Controls.kSlowRotateAxis) > 0.5){
            speedRotation *= 0.5;
        }

        speedForward = Maths.deadband(speedForward, 0.08);
        speedRotation = Maths.deadband(speedRotation, 0.08);

        // Aim to vision target on button press
        if (m_joystick.getRawButton(IDs.Controls.kAlignToTargetButton)) {
            // Turn on limelight
            enableLimelight(true);

            // Turn to target and allow distance adjustments from joystick when aligned
            if (turnToTarget()){
                setDrive(m_joystick.getRawAxis(IDs.Controls.kDriveForwardAxis));
                nextLEDstate = LEDmode.GREEN;
            } else {
                nextLEDstate = LEDmode.ORANGE;
            }


        } else {
            // Turn off limelight
            enableLimelight(false);

            m_drive.arcadeDrive(-speedRotation, speedForward);
        }

        setLEDs(nextLEDstate);
    }

    @Override
    public void disabledInit() {
        enableLimelight(false);
    }

    @Override
    public void disabledPeriodic() {
        enableLimelight(false);
    }

    @Override
    public void testInit(){
        stopAll();
    }

    @Override
    public void testPeriodic(){

        // TEST MODE - Toggle limelight driver camera on button press
        if (m_joystick.getRawButtonPressed(IDs.Controls.kToggleCamButton)) {
            toggleDriverCam();
        }

        // TEST MODE - Run shooter on trigger
        if (m_gamepad.getRawAxis(IDs.Controls.kDriveShooterAxis) > 0.2) {
            m_shooterMotorL.set(0.2);
            m_shooterMotorR.set(0.2);
        } else {
            m_shooterMotorL.set(0);
            m_shooterMotorR.set(0);
        }

        // TEST MODE - Run intake in on button press and out on button press
        if (m_joystick.getRawButton(IDs.Controls.kRunIntakeNormalButton) ) {
            m_intakeMotor.set(0.2);
        } else if (m_joystick.getRawButton(IDs.Controls.kRunIntakeReverseButton)) {
            m_intakeMotor.set(-0.2);
        } else {
            m_intakeMotor.set(0);
        }

        // TEST MODE - Toggle intake arm position on button press
        if (m_joystick.getRawButtonPressed(IDs.Controls.kToggleIntakePositionButton)) {
            if (!m_intakeSolenoid.get().equals(Value.kForward)) {
                setIntakeArm(Value.kForward);
            } else {
                setIntakeArm(Value.kReverse);
            }
        }

        // TEST MODE - Run esophagus on button press
        if (m_gamepad.getRawButton(IDs.Controls.kRunEsophagusAxis)) {
            enableEsophagus(true);
        } else {
            enableEsophagus(false);
        }

        // TEST MODE - Run hopper if intake or esophagus is running otherwise always run on button press
        if (m_joystick.getPOV() == (IDs.Controls.kRunHopperPOV)){
            enableHopper(true);
        } else if (m_joystick.getRawButton(IDs.Controls.kRunIntakeNormalButton) ||
                   m_joystick.getRawButton(IDs.Controls.kRunIntakeReverseButton) || 
                   m_gamepad.getRawButton(IDs.Controls.kRunEsophagusAxis)) {
            enableHopper(true);
        } else {
            enableHopper(false);
        }

    }
    

    public void setAutonStage(int stage) {
        setAutonStage(stage, false);
    }

    public void setAutonStage(int stage, boolean stopAll) {

        resetEncoders();
        m_autonTimer.reset();
        m_gyro.reset();

        m_autonPIDerrSum = 0;

        if(stopAll) stopAll();

        m_autonStage = stage;
    }

    public void resetEncoders() {
        m_driveFL.getSensorCollection().setIntegratedSensorPosition(0, 0);
        m_driveFR.getSensorCollection().setIntegratedSensorPosition(0, 0);
        // m_driveRL.getSensorCollection().setIntegratedSensorPosition(0, 0);
        // m_driveRR.getSensorCollection().setIntegratedSensorPosition(0, 0);
    }

    public void stopAll() {
        // Stop all drive motors
        setDrive(0);

        // Stop shooter
        enableShooter(false);

        // Stop intake/esophagus/hopper
        setIntake(MotorSpeed.STOPPED);
        enableEsophagus(false);
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
        final double kI = 0.05;
        final double integralLimit = 10;

        double dt = Timer.getFPGATimestamp();

        double setpoint = degrees;

        if (setpoint > 180) {
            setpoint = -(360 - setpoint);
        } else if (setpoint < -180) {
            setpoint = 360 + setpoint;
        }

        double err = setpoint - gyroAngle;

        if(Math.abs(err) < integralLimit){
            m_autonPIDerrSum += err * dt;
        }

        double speed = err * kP;

        if (Math.abs(err) > 2 && m_autonTimer.get() < timeout) {
            setDriveRotate(-speed);
        } else {
            complete = true;
        }

        m_autonPIDlastTimestamp = Timer.getFPGATimestamp();

        return complete;
    }

    public boolean turnToTarget() {

        boolean complete = false;

        double tx = getLimelightValue("tx") - 5;

        if (getLimelightValue("camMode") == 1) {
            toggleDriverCam();
        }

        if (Math.abs(tx) > 4.0) {
            setDriveRotate(-(tx / 26) * 0.5);
        } else {
            complete = true;
        }

        return complete;
    }

    public boolean isTurnedToTarget(){
        boolean ans = false;

        if (Math.abs(getLimelightValue("tx")) < 4.0){
            ans = true;
        }

        return ans;
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
        // m_driveRL.set(speedL);
        // m_driveRR.set(speedR);
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

    public boolean isShooterAtSpeed(){
        boolean ans = false;

        if(m_shooterMotorR.get() == 0 || m_shooterMotorL.get() == 0){
            ans = false;
        } else if (m_shooterMotorR.getEncoder().getVelocity() <= -4000){
            ans = true;
        }

        return ans;
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

    public void enableEsophagus(boolean enabled) {
        if (enabled) {
            m_esophagusMotor.set(kIndexSpeed);
        } else {
            m_esophagusMotor.set(0);
        }
    }

    public boolean enableEsophagusAuto(boolean override){

        boolean run = false;

        // Run esophagus when powercell is at esophagus entry and not at shooter, or if shooter is at speed, override all if true
        if (override){
            enableEsophagus(true);
            run = true;
        } else if (m_esophagusSensorBottom.get() && !m_esophagusSensorTop.get()) {
            enableEsophagus(true);
            run = true;
        } else {
            enableEsophagus(false);
        }

        return run;
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

        double value = 0.0;

        if (entry.equals("tDistance")) {
            value = (kTargetHeight - kLimelightHeight) / Math.tan(Math.toRadians(kLimelightAngle + getLimelightValue("ty")));
        } else {
            value = NetworkTableInstance.getDefault().getTable("limelight").getEntry(entry).getDouble(0.0);
        }

        return value;
    }

    public void setLimelightValue(String entry, double value) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry(entry).setDouble(value);
    }

    public void setLEDs(LEDmode mode) {
        ledState = mode;
        if (mode == LEDmode.DEFAULT) {
            // m_ledController.set(0);
        } else if (mode == LEDmode.RGB) {
            // m_ledController.set(-0.99);
        } else if (mode == LEDmode.RED) {
            // m_ledController.set(0.61);
        } else if (mode == LEDmode.GREEN) {
            // m_ledController.set(0.77);
        } else if (mode == LEDmode.SHOT_A) {
            // m_ledController.set(0.13);
        } else if (mode == LEDmode.SHOT_B) {
            // m_ledController.set(0.33);
        } else if (mode == LEDmode.ORANGE) {
            // m_ledController.set(0.65);
        }
    }

    public void setDriveNeutralMode(NeutralMode mode) {
        m_driveFL.setNeutralMode(mode);
        m_driveFR.setNeutralMode(mode);
        // m_driveRL.setNeutralMode(mode);
        // m_driveRR.setNeutralMode(mode);
    }

}
