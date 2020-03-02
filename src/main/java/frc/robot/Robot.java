
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Robot extends TimedRobot {

  // Driver Input
  private Joystick m_joystick = new Joystick(IDs.kJoystick);
  private Joystick m_gamepad = new Joystick(IDs.kGamepad);

  // Drive Motors
  public WPI_TalonFX m_driveFL = new WPI_TalonFX(IDs.kFrontLeftChannel);
  public WPI_TalonFX m_driveRL = new WPI_TalonFX(IDs.kRearLeftChannel);
  public WPI_TalonFX m_driveFR = new WPI_TalonFX(IDs.kFrontRightChannel);
  public WPI_TalonFX m_driveRR = new WPI_TalonFX(IDs.kRearRightChannel);

  // Game Piece Manipulation
  private WPI_VictorSPX m_intakeMotor = new WPI_VictorSPX(IDs.kIntake);
  private WPI_VictorSPX m_indexMotor = new WPI_VictorSPX(IDs.kIndexer);
  private WPI_VictorSPX m_hopperMotor = new WPI_VictorSPX(IDs.kHopper);

  // - Shooter
  private CANSparkMax m_shooterMotorL = new CANSparkMax(IDs.kShooterL, MotorType.kBrushless);
  private CANSparkMax m_shooterMotorR = new CANSparkMax(IDs.kShooterR, MotorType.kBrushless);

  // Climber
  private WPI_TalonSRX m_climberLiftMotor = new WPI_TalonSRX(24);
  private WPI_TalonSRX m_climberMotorR = new WPI_TalonSRX(23);
  private WPI_TalonSRX m_climberMotorL = new WPI_TalonSRX(25);
  private WPI_VictorSPX m_climberSliderMotor = new WPI_VictorSPX(35);

  // Sensors
  private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
  private final I2C.Port m_i2c = I2C.Port.kOnboard;

  // Internal Game Piece Sensors
  private DigitalInput m_indexSensor0 = new DigitalInput(0);
  private DigitalInput m_indexSensor1 = new DigitalInput(1);
  private DigitalInput m_indexSensor2 = new DigitalInput(2);

  // Pneumatics
  private DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(IDs.kPneumatics, 0, 1);

  // LED Controller
  private Spark m_ledController = new Spark(0);

  // Autonomous
  private Joystick m_autonSelector = new Joystick(IDs.kDSSwitches);
  private Timer m_autonTimer = new Timer();
  private int m_autonMode = 0;
  private int m_autonStage = 0;
  private final double kEncoderTicksPerInch = (2048 * 10.71) / (Math.PI * 6);

  // Field Element Dimensions
  private final double kTargetHeight = 98.25;

  // Robot Dimensions
  private final double kLimelightHeight = 41.0; // TODO: Update when limelight mounted
  private final double kLimelightAngle = -89.0; // TODO: Update when limelight mounted

  // Motor Output Speeds/Limits
  private final double kDriveSpeedMultipliter      = 1.0;
  private final double kDriveSpeedMultipliterTurbo = 1.0;
  private final double kMaxDriveOutput   = 1.0;
  private final double kShooterSpeed     = -0.95; // at one tick out speed 0.685l // < Not sure what that meant but I'm keeping it there in case it's important - JD
  private final double kIndexSpeed       = -0.9;
  private final double kHopperSpeed      = -0.3;
  private final double kIntakeSpeed      = 1.0;
  private final double kClimberLiftSpeed = 0.7;
  private final double kClimberSpeed     = 1.0;

  private static enum MotorSpeed {
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

    setDriveNeutralMode(NeutralMode.Coast);

    // Reset gyro
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

    // TESTING - Put values on SmartDashboard
    SmartDashboard.putNumber("limelight TX", getLimelightValue("tx")); // Difference of X axis angles between crosshairs
    SmartDashboard.putNumber("limelight TY", getLimelightValue("ty")); // Difference of Y axis angles between crosshairs
    SmartDashboard.putNumber("limelight TA", getLimelightValue("ta")); // Area of target
    SmartDashboard.putNumber("limelight cam", getLimelightValue("camMode")); // Camera mode [0 = VISION | 1 = DRIVER
                                                                             // CAM]
    SmartDashboard.putNumber("limelight led", getLimelightValue("ledMode")); // LED Mode [0 = DEFAULT | 1 = FORCE OFF |
                                                                             // 2 = FORCE BLINK | 3 = FORCE ON]
    SmartDashboard.putNumber("target distance", getLimelightValue("tDistance")); // Distance from target

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

    m_autonMode = 0;

    for (int i = 3; i < 8; i++) {
      if (m_autonSelector.getRawButton(i)) {
        m_autonMode = i - 3;
        break;
      }
    }
  }

  @Override
  public void autonomousPeriodic() {

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

      // switch (m_autonStage) {
      // case (0): // Stage 0: Start ramping up shooter
      //   enableShooter(true);

      //   if (m_shooterMotorR.getEncoder().getVelocity() <= -4000 || m_autonTimer.get() > 5) {
      //     setAutonStage(1);
      //   }

      //   break;
      // case (1): // Stage 1: Fire starting payload
      //   enableShooter(true);

      //   //if (turnToTarget()) {
      //     enableIndexer(true);
      //   //}

      //   if (m_autonTimer.get() > 3.2) {
      //     setAutonStage(2);
      //   }

      //   break;
      // case (2): // Stage 2: Stop shooter and turn to face trench run
      //   enableIndexer(false);
      //   enableShooter(false);

      //   if (turnTo(130, 2)) {
      //     setAutonStage(3);
      //   }

      //   break;
      // case (3): // Stage 3: Move to trench run
      //   if (moveTo(100, 3)) {
      //     setAutonStage(4);
      //   }

      //   break;
      // case (4): // Stage 4: Align to trench run
      //   if (turnTo(45, 2)) {
      //     setAutonStage(5);
      //   }

      //   break;
      // case (5): // Stage 5: Drop intake
      //   stopAll();
      //   setIntakeArm(Value.kForward);

      //   if (m_autonTimer.get() >= 2) {
      //     setAutonStage(6);
      //   }

      //   break;
      // case (6): // Stage 6: Start intake and move forward, collecting powercells
      //   setIntake(MotorSpeed.FORWARD);

      //   if (moveTo(123, 4)) {
      //     setAutonStage(7);
      //   }

      //   break;
      // case (7): // Stage 7: Stop intake and move back to start of trench run
      //   setIntake(MotorSpeed.FORWARD);

      //   if (moveTo(-123, 4)) {
      //     setAutonStage(8);
      //   }

      //   break;
      // default: // Default to stop all motors and stop timer
      //   stopAll();
      //   m_autonTimer.stop();
      //   m_autonTimer.reset();
      //   break;
      // }
    }
  }

  @Override
  public void teleopInit() {
  }

  /*
   * Control Scheme
   *
   * Drive: joystick Y for forward-backward, twist for rotating 
   * Turbo: fire 4
   * Align to Vision Target: joystick fire 3
   * Toggle Limelight Driver Cam: joystick fire 5 
   * Intake Deploy Toggle: fire 6 
   * Drive Intake: trigger Drive Intake Reversed: fire 2
   * 
   * Drive Intake Inverted: gamepad left bumper 
   * Drive Esophagus: gamepad Y Drive
   * Climber Lift Up: gamepad direction up 
   * Drive Climber Lift Down: gamepad direction down 
   * Drive Climber Up: gamepad direction right
   * 
   */

  @Override
  public void teleopPeriodic() {

    // Toggle limelight driver camera on joystick fire 3 pressed
    if (m_joystick.getRawButtonPressed(IDs.JoystickIDs.FIRE_5)) {
      toggleDriverCam();
    }

    // Enable shooter on gamepad right trigger pulled
    if (m_gamepad.getRawAxis(IDs.ControllerIDs.RIGHT_TRIGGER_AXIS) > 0.2) {
      enableShooter(true);
    } else {
      enableShooter(false);
    }

    // Enable intake/hopper in on gamepad left trigger pulled and out on gamepad
    // left bumper pressed (if intake arm is extended)
    if (m_joystick.getRawButton(IDs.JoystickIDs.TRIGGER_BUTTON) && m_intakeSolenoid.get().equals(Value.kForward)) {
      setIntake(MotorSpeed.FORWARD);
      enableHopper(true);
    } else if (m_joystick.getRawButton(IDs.JoystickIDs.FIRE_2) && m_intakeSolenoid.get().equals(Value.kForward)) {
      setIntake(MotorSpeed.REVERSE);
      enableHopper(false);
    } else {
      setIntake(MotorSpeed.STOPPED);
      enableHopper(false);
    }

    // Toggle intake arm position on gamepad right bumper pressed
    if (m_joystick.getRawButtonPressed(IDs.JoystickIDs.FIRE_6)) {
      if (!m_intakeSolenoid.get().equals(Value.kForward)) {
        setIntakeArm(Value.kForward);
      } else {
        setIntakeArm(Value.kReverse);
      }
    }

    // Enable esophagus on gamepad Y button pressed
    if (m_gamepad.getRawButton(IDs.ControllerIDs.Y_BUTTON)) {
      enableIndexer(true);
    } else {
      enableIndexer(false);
    }

    // Drive climber on gamepad directional right pressed
    if (m_gamepad.getPOV() == 90) {

      setClimberLift(0);
      m_climberLiftMotor.setNeutralMode(NeutralMode.Coast);

      setClimber(kClimberSpeed);

    } else {

      setClimber(0);
      m_climberLiftMotor.setNeutralMode(NeutralMode.Brake);

      // Drive climber lift up on gamepad directional up pressed, down on gamepad
      // directional down pressed
      if (m_gamepad.getPOV() == 0) {
        setClimberLift(kClimberLiftSpeed);

      } else if (m_gamepad.getPOV() == 180) {
        setClimberLift(-kClimberLiftSpeed);

      } else {
        setClimberLift(0);
      }
    }

    // Drive climb slider with gamepad left joystick X axis
    m_climberSliderMotor.set(m_gamepad.getRawAxis(IDs.ControllerIDs.LEFT_X_AXIS));

    // Speed Init
    double speedL = 0.0;
    double speedR = 0.0;

    // Controls : Joystick forward-backward + twist
    // Add joystick forward-backward
    speedL += m_joystick.getRawAxis(IDs.JoystickIDs.Y_AXIS);
    speedR += m_joystick.getRawAxis(IDs.JoystickIDs.Y_AXIS);

    // Add joystick twist
    speedL -= m_joystick.getRawAxis(IDs.JoystickIDs.THROTTLE_AXIS);
    speedR += m_joystick.getRawAxis(IDs.JoystickIDs.THROTTLE_AXIS);

    // TESTING - Put everything on the SmartDashboard to figure out what the h e c k is going wrong
    SmartDashboard.putNumber("joystick Y", m_joystick.getRawAxis(IDs.JoystickIDs.Y_AXIS));
    SmartDashboard.putNumber("joystick twist", m_joystick.getRawAxis(IDs.JoystickIDs.THROTTLE_AXIS));
    SmartDashboard.putNumber("speedL", speedL);
    SmartDashboard.putNumber("speedL", speedR);

    // Main Drive
    // Aim to vision target on joystick fire 2 pressed, else drive standard
    if (m_joystick.getRawButton(IDs.JoystickIDs.FIRE_3)) {
      // Turn on limelight
      enableLimelight(true);

      // Turn to target and allow distance adjustments from joystick when aligned
      if (turnToTarget()) {
        setDrive(m_joystick.getRawAxis(IDs.JoystickIDs.Y_AXIS) * kDriveSpeedMultipliter);
      }

    } else {
      // Turn off limelight
      enableLimelight(false);

      // Drive with turbo speed on joystick trigger button pressed, else drive normal
      // speed
      if (m_joystick.getRawButton(IDs.JoystickIDs.FIRE_4)) {
        setDrive(speedL * kDriveSpeedMultipliterTurbo, speedR * kDriveSpeedMultipliterTurbo);
      } else {
        setDrive(speedL * kDriveSpeedMultipliter, speedR * kDriveSpeedMultipliter);
      }
    }
  }

  @Override
  public void disabledInit() {
    enableLimelight(false);
  }

  private void setAutonStage(int stage) {

    resetEncoders();
    m_autonTimer.reset();
    m_gyro.reset();

    m_autonStage = stage;
  }

  private void resetEncoders() {
    m_driveFR.getSensorCollection().setIntegratedSensorPosition(0, 0);
    m_driveFL.getSensorCollection().setIntegratedSensorPosition(0, 0);
    m_driveRR.getSensorCollection().setIntegratedSensorPosition(0, 0);
    m_driveRL.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

  private void stopAll() {
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
  }

  private boolean moveTo(double in, double timeout) {

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

  private boolean turnTo(double degrees, double timeout) {

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

  private boolean turnToTarget() {

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

  private void setDriveRotate(double speed) {
    setDrive(speed, -speed);
  }

  private void setDrive(double speed) {
    setDrive(speed, speed);
  }

  private void setDrive(double speedL, double speedR) {
    m_driveFL.set(limitSpeed(speedL));
    m_driveFR.set(limitSpeed(speedR));
    m_driveRL.set(limitSpeed(speedL));
    m_driveRR.set(limitSpeed(speedR));
  }

  private void enableShooter(boolean enabled) {
    if (enabled) {
      m_shooterMotorL.set(kShooterSpeed);
      m_shooterMotorR.set(kShooterSpeed);
    } else {
      m_shooterMotorL.set(0);
      m_shooterMotorR.set(0);
    }
  }

  private void setIntakeArm(Value position) {
    if (position.equals(Value.kForward)) {
      m_intakeSolenoid.set(Value.kForward);
    } else if (position.equals(Value.kReverse)) {
      m_intakeSolenoid.set(Value.kReverse);
    } else {
      m_intakeSolenoid.set(Value.kOff);
    }
  }

  private void setIntake(MotorSpeed direction) {
    if (direction == MotorSpeed.FORWARD) {
      m_intakeMotor.set(kIntakeSpeed);
    } else if (direction == MotorSpeed.REVERSE) {
      m_intakeMotor.set(-kIntakeSpeed);
    } else {
      m_intakeMotor.set(0);
    }
  }

  private void enableHopper(boolean enabled) {
    if (enabled) {
      m_hopperMotor.set(kHopperSpeed);
    } else {
      m_hopperMotor.set(0);
    }
  }

  private void enableIndexer(boolean enabled) {
    if (enabled) {
      m_indexMotor.set(kIndexSpeed);
    } else {
      m_indexMotor.set(0);
    }
  }

  private void setClimberLift(double speed) {
    m_climberLiftMotor.set(speed);
  }

  private void setClimber(double speed) {
    m_climberMotorL.set(speed);
    m_climberMotorR.set(speed);
  }

  private void enableLimelight(boolean enabled) {
    if (enabled) {
      setLimelightValue("ledMode", 0);
    } else {
      setLimelightValue("ledMode", 1);
    }
  }

  private void toggleDriverCam() {
    double currentMode = getLimelightValue("camMode");

    if (currentMode == 0) {
      setLimelightValue("camMode", 1);
    } else if (currentMode == 1) {
      setLimelightValue("camMode", 0);
    }
  }

  private double getLimelightValue(String entry) {

    double value = -1.0;

    if (entry.equals("tDistance")) {
      value = (kTargetHeight - kLimelightHeight) / Math.tan(Math.toRadians(kLimelightAngle + getLimelightValue("ty")));
    } else {
      value = NetworkTableInstance.getDefault().getTable("limelight").getEntry(entry).getDouble(-1.0);
    }

    return value;
  }

  private void setLimelightValue(String entry, double value) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry(entry).setDouble(value);
  }

  /**
   * 
   * @param mode 0 = default | 1 = rainbow | 2 = solid red | 3 = solid green | 4 =
   *             primary color shot
   */
  private void setLEDs(int mode) {
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

  private void setDriveNeutralMode(NeutralMode mode) {
    m_driveFR.setNeutralMode(mode);
    m_driveFL.setNeutralMode(mode);
    m_driveRR.setNeutralMode(mode);
    m_driveRL.setNeutralMode(mode);
  }

  private double limitSpeed(double d) {
    if (d > kMaxDriveOutput) {
      d = kMaxDriveOutput;
    } else if (d < -kMaxDriveOutput) {
      d = -kMaxDriveOutput;
    }
    return d;
  }

}