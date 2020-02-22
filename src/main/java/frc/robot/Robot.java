
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
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
  private WPI_TalonFX driveFL = new WPI_TalonFX(IDs.kFrontLeftChannel);
  private WPI_TalonFX driveRL = new WPI_TalonFX(IDs.kRearLeftChannel);
  private WPI_TalonFX driveFR = new WPI_TalonFX(IDs.kFrontRightChannel);
  private WPI_TalonFX driveRR = new WPI_TalonFX(IDs.kRearRightChannel);

  // Game Piece Manipulation
  private WPI_VictorSPX intakeMotor = new WPI_VictorSPX(IDs.kIntake);
  private WPI_VictorSPX indexMotor = new WPI_VictorSPX(IDs.kIndexer);
  private WPI_VictorSPX hopperMotor = new WPI_VictorSPX(IDs.kHopper);
  
  private CANSparkMax shooterMotorL = new CANSparkMax(IDs.kShooterL, MotorType.kBrushless);
  private CANSparkMax shooterMotorR = new CANSparkMax(IDs.kShooterR, MotorType.kBrushless);

  // private WPI_TalonSRX climberMotorL = new WPI_TalonSRX(-1);
  // private WPI_TalonSRX climberMotorR = new WPI_TalonSRX(-1);
  // private WPI_TalonSRX climberLiftMotor = new WPI_TalonSRX(-1);

  // Field Element Dimentional Information
  private static final double kTargetHeight = 98.25;

  // Motor Output Limits
  private double speedMultiplier = 0.5;
  private double speedMultiplierTurbo = 0.6;
  private double shooterSpeed = 0.9; // at one tick out speed 0.685l
  private final double kMaxDriveOutput = 0.6;
  
  private double indexMotorSpeed = 0.0;
  private double intakeMotorSpeed = 0.0;
  private double hopperMotorSpeed = 0.0;
  
  // Sensors
  private ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  // Pneumatics  
  private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(IDs.kPneumatics, 0, 1);

  // Encoders
  private double encoderPosition = driveFR.getSensorCollection().getIntegratedSensorPosition();

  // Internal Game Piece Sensors
  // private DigitalInput pcSensor0 = new DigitalInput(0);
  // private DigitalInput pcSensor1 = new DigitalInput(1);
  // private DigitalInput pcSensor2 = new DigitalInput(2);

  private Spark ledController = new Spark(0);

  @Override
  public void robotInit() {
    
    // Invert left drive motors
    driveFL.setInverted(true);
    driveRL.setInverted(true);

    // Reset encoder positions
    driveFR.getSensorCollection().setIntegratedSensorPosition(0, 0);
    driveFL.getSensorCollection().setIntegratedSensorPosition(0, 0);
    driveRR.getSensorCollection().setIntegratedSensorPosition(0, 0);
    driveRL.getSensorCollection().setIntegratedSensorPosition(0, 0);

    // Set rear drive motors to follow front drive motors
    driveRL.follow(driveFL);
    driveRR.follow(driveFR);

    // Invert right climber motor
    // climberMotorR.setInverted(true);

    // Reset gyro 
    m_gyro.reset();
  }

  @Override
  public void robotPeriodic() {

    // Put front right encoder position on SmartDashboard for testing
    encoderPosition = driveFR.getSensorCollection().getIntegratedSensorPosition();
    SmartDashboard.putNumber("raw encoder value", encoderPosition / (2048 * 10.71));

    // Put gyro angle on SmartDashboard for testing
    SmartDashboard.putNumber("gyro angle", m_gyro.getAngle());
  }

  @Override
  public void autonomousInit() {
    // Reset encoder positions
    driveFR.getSensorCollection().setIntegratedSensorPosition(0, 0);
    driveFL.getSensorCollection().setIntegratedSensorPosition(0, 0);
    driveRR.getSensorCollection().setIntegratedSensorPosition(0, 0);
    driveRL.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // Set inital solenoid value
    intakeSolenoid.set(Value.kReverse);
  }

  @Override
  public void teleopPeriodic() {

    // Speed Init
    double speedL = 0.0;
    double speedR = 0.0;

    // Controls : Joystick forward-backward + twist
    // Add joystick forward-backward
    speedL += m_joystick.getRawAxis(IDs.JoystickIDs.Y_AXIS);
    speedR += m_joystick.getRawAxis(IDs.JoystickIDs.Y_AXIS);

    // Add joystick twist
    speedL -= m_joystick.getRawAxis(IDs.JoystickIDs.TWIST_AXIS) * 0.75;
    speedR += m_joystick.getRawAxis(IDs.JoystickIDs.TWIST_AXIS) * 0.75;

    // Limelight network table
    NetworkTable llTable = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry llTableTX = llTable.getEntry("tx");      // Difference of X axis angles between crosshairs
    NetworkTableEntry llTableTY = llTable.getEntry("ty");      // Difference of Y axis angles between crosshairs
    NetworkTableEntry llTableTA = llTable.getEntry("ta");      // Area of target
    NetworkTableEntry llTableCM = llTable.getEntry("camMode"); // Camera mode [0 = VISION | 1 = DRIVER CAM]
    NetworkTableEntry llTableLM = llTable.getEntry("ledMode"); // LED Mode [0 = DEFAULT | 1 = FORCE OFF]

    // Parse network table data
    double tx = llTableTX.getDouble(0.0);
    double ty = llTableTY.getDouble(0.0);
    double ta = llTableTA.getDouble(0.0);
    double cm = llTableCM.getDouble(-1.0);
    double lm = llTableLM.getDouble(-1.0);

    // Distance from limelight to target base using formula d = (h2 - h1) / tan(a1 + a2)
    double tDistance = (kTargetHeight - 22.0) / Math.tan(Math.toRadians(20.0 + ty));

    // Put limelight values on Smart Dashboard
    SmartDashboard.putNumber("Limelight TX", tx);
    SmartDashboard.putNumber("Limelight TY", ty);
    SmartDashboard.putNumber("Limelight TA", ta);
    SmartDashboard.putNumber("Target Distance", tDistance);
    
    // Toggle limelight driver camera on joystick fire 3 pressed
    if (m_joystick.getRawButtonPressed(IDs.JoystickIDs.FIRE_3)) {
      if (cm == 1) {
        llTableCM.setNumber(0);
      } else if (cm == 0) {
        llTableCM.setNumber(1);
      }
    }

    // Drive shooter motors on gamepad Y button pressed
    if(m_gamepad.getRawButton(IDs.ControllerIDs.Y_BUTTON)){
      shooterMotorL.set(-shooterSpeed);
      shooterMotorR.set(-shooterSpeed);
    } else {
      shooterMotorL.set(0);
      shooterMotorR.set(0);
    }

    // Toggle intake solenoid position on gamepad B button pressed
    if(m_gamepad.getRawButtonPressed(IDs.ControllerIDs.B_BUTTON)){
      if(!intakeSolenoid.get().equals(Value.kForward)){
        intakeSolenoid.set(Value.kForward);      
      } else {
        intakeSolenoid.set(Value.kReverse);
      }
    }

    // Reset intake/hopper/index motor speeds
    intakeMotorSpeed = 0.0;
    hopperMotorSpeed = 0.0;
    indexMotorSpeed = 0.0;
    
    // Enable intake/hopper/index motors on gamepad A button pressed
    if(m_gamepad.getRawButton(IDs.ControllerIDs.A_BUTTON)){

      // Invert speeds on gamepad left bumper pressed
      if(!m_gamepad.getRawButton(IDs.ControllerIDs.LEFT_BUMPER_BUTTON)){

        // Enable index motors if power cell is not detected by the top index sensor and the shooter speed isn't 0
        //if(!pcSensor2.get() || shooterMotorL.get() != 0){
          indexMotorSpeed = -1.0;
        //}

        intakeMotorSpeed = 1.0;
        hopperMotorSpeed = -0.3;

      } else { 
        intakeMotorSpeed = -1.0;
        hopperMotorSpeed = -0.3;
        indexMotorSpeed = 1.0;
      }
      
    }

    // Drive intake/hopper motors
    intakeMotor.set(intakeMotorSpeed);
    hopperMotor.set(hopperMotorSpeed);
    indexMotor.set(indexMotorSpeed);

    // if(m_gamepad.getRawAxis(IDs.ControllerIDs.LEFT_TRIGGER_AXIS) > 0.08){
    //   climberMotorR.set(m_gamepad.getRawAxis(IDs.ControllerIDs.LEFT_TRIGGER_AXIS));
    //   climberMotorL.set(m_gamepad.getRawAxis(IDs.ControllerIDs.LEFT_TRIGGER_AXIS));
    // }

    // if(m_gamepad.getRawAxis(IDs.ControllerIDs.RIGHT_TRIGGER_AXIS) > 0.08){
    //   climberMotorR.set(-m_gamepad.getRawAxis(IDs.ControllerIDs.RIGHT_TRIGGER_AXIS));
    //   climberMotorL.set(-m_gamepad.getRawAxis(IDs.ControllerIDs.RIGHT_TRIGGER_AXIS));
    // }
    
    // Main drive
    if (m_joystick.getRawButton(IDs.JoystickIDs.FIRE_2)) {
      // Turn on limelight
      llTableLM.setNumber(0);

      // Turn to target when horizontal angle from bot crosshair to target center is greater than 4 degrees
      if (Math.abs(tx) > 4.0) {
        turn(-((tx / 26) * 0.6));
      }
    } else {
      // Turn off limelight
      llTableLM.setNumber(1);

      // Drive with turbo speed on joystick trigger button pressed, else drive normal
      if(m_joystick.getRawButton(IDs.JoystickIDs.TRIGGER_BUTTON)){
        move(speedL * speedMultiplierTurbo, speedR * speedMultiplierTurbo);
      } else {
        move(speedL * speedMultiplier, speedR * speedMultiplier);
      }

    }

    // Put distance moved in inches based on front right encoder value
    SmartDashboard.putNumber("distance moved", -driveFR.getSensorCollection().getIntegratedSensorPosition() / 1163.64);
  }

  @Override
  public void disabledInit() {
    // Turn off limelight
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
  }

  private void turn(double speed) {
    move(speed, -speed);
  }

  private void move(double speed) {
    move(speed, speed);
  }

  private void move(double speedL, double speedR) {
    driveFL.set(normalize(speedL));
    driveFR.set(normalize(speedR));
  }

  /** Limit drive power to maximum speed
   * @param d : number to be limited
   * 
   * NOTE: The built in Math function did a spooky so this is what were using now, k? k.
   */
  private double normalize(double d) {
    if(d > kMaxDriveOutput){
      d = kMaxDriveOutput;
    } else if (d < -kMaxDriveOutput){
      d = -kMaxDriveOutput;
    }
    return d;
  }

}