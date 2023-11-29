// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Arm;

public class Robot extends TimedRobot {
  /*
   * Autonomous selection options.
   */
  private static final String kNothingAuto = "do nothing";
  private static final String kConeAuto = "cone";
  private static final String kCubeAuto = "cube";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private AddressableLED led = new AddressableLED(0);
  private AddressableLEDBuffer length = new AddressableLEDBuffer(238); 
   /*
   * Drive motor controller instances.
   * 
   * Change the id's to match your robot.
   * Change kBrushed to kBrushless if you are using NEO's.
   * Use the appropriate other class if you are using different controllers.
   */
  

  /*
   * Mechanism motor controller instances.
   * 
   * Like the drive motors, set the CAN id's to match your robot or use different
   * motor controller classses (TalonFX, TalonSRX, Spark, VictorSP) to match your
   * robot.
   * 
   * The arm is a NEO on Everybud.
   * The intake is a NEO 550 on Everybud.
   */
  // TalonSRX arm = new TalonSRX(5);
  // TalonSRX arm2 = new TalonSRX(7);
  // VictorSPX intake = new VictorSPX(6);

  /**
   * The starter code uses the most generic joystick class.
   * 
   * The reveal video was filmed using a logitech gamepad set to
   * directinput mode (switch set to D on the bottom). You may want
   * to use the XBoxController class with the gamepad set to XInput
   * mode (switch set to X on the bottom) or a different controller
   * that you feel is more comfortable.
   */
  // Joystick j = new Joystick(0);
  XboxController j = new XboxController(0);
  /*
   * Magic numbers. Use these to adjust settings.
   */

  /**
   * How many amps the arm motor can use.
   */

  /**
   * Percent output to run the arm up/down at
   */

  /**
   * How many amps the intake can use while picking up
   */

  /**
   * How many amps the intake can use while holding
   */

  /**
   * Percent output for intaking
   */

  /**
   * Percent output for holding
   */

  /**
   * Time to extend or retract arm in auto
   */

  /**
   * Time to throw game piece in auto
   */

  /**
   * Time to drive back in auto
   */

  /**
   * Speed to drive backwards in auto
   */

  /**
   * This method is run once when the robot is first started up.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("do nothing", kNothingAuto);
    m_chooser.addOption("cone and mobility", kConeAuto);
    m_chooser.addOption("cube and mobility", kCubeAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    led.setLength(length.getLength());
    led.start();

    /*
     * You will need to change some of these from false to true.
     * 
     * In the setDriveMotors method, comment out all but 1 of the 4 calls
     * to the set() methods. Push the joystick forward. Reverse the motor
     * if it is going the wrong way. Repeat for the other 3 motors.
     */
   

    /*
     * Set the arm and intake to brake mode to help hold position.
     * If either one is reversed, change that here too. Arm out is defined
     * as positive, arm in is negative.
     */
    
  }

  /**
   * Calculate and set the power to apply to the left and right
   * drive motors.
   * 
   * @param forward Desired forward speed. Positive is forward.
   * @param turn    Desired turning speed. Positive is counter clockwise from
   *                above.
   */
  // public void setDriveMotors(double forward, double turn) {
  //   SmartDashboard.putNumber("drive forward power (%)", forward);
  //   SmartDashboard.putNumber("drive turn power (%)", turn);

  //   /*
  //    * positive turn = counter clockwise, so the left side goes backwards
  //    */
  //   double left = forward - turn;
  //   double right = forward + turn;

  //   SmartDashboard.putNumber("drive left power (%)", left);
  //   SmartDashboard.putNumber("drive right power (%)", right);

  //   // see note above in robotInit about commenting these out one by one to set
  //   // directions.
    
  // }

  /**
   * Set the arm output power. Positive is out, negative is in.
   * 
   * @param percent
   */
  

  /**
   * Set the arm output power.
   * 
   * @param percent desired speed
   * @param amps    current limit
   */
  

  /**
   * This method is called every 20 ms, no matter the mode. It runs after
   * the autonomous and teleop specific period methods.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());

    for (var i = 0; i < length.getLength(); i++){
      //rgb(0, 43, 175)
      length.setRGB(i,0,43,175);
    }
    led.setData(length);
  }

  double autonomousStartTime;
  double autonomousIntakePower;

  @Override
  public void autonomousInit() {


    // m_autoSelected = m_chooser.getSelected();
    // System.out.println("Auto selected: " + m_autoSelected);

    // if (m_autoSelected == kConeAuto) {
    //   autonomousIntakePower = INTAKE_OUTPUT_POWER;
    // } else if (m_autoSelected == kCubeAuto) {
    //   autonomousIntakePower = -INTAKE_OUTPUT_POWER;
    // }

    // autonomousStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {
    // if (m_autoSelected == kNothingAuto) {
    //   setArmMotor(0.0);
    //   setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    //   setDriveMotors(0.0, 0.0);
    //   return;
    // }

    // double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;

    // if (timeElapsed < ARM_EXTEND_TIME_S) {
    //   setArmMotor(ARM_OUTPUT_POWER);
    //   setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    //   setDriveMotors(0.0, 0.0);
    // } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S) {
    //   setArmMotor(0.0);
    //   setIntakeMotor(autonomousIntakePower, INTAKE_CURRENT_LIMIT_A);
    //   setDriveMotors(0.0, 0.0);
    // } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S) {
    //   setArmMotor(-ARM_OUTPUT_POWER);
    //   setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    //   setDriveMotors(0.0, 0.0);
    // } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + AUTO_DRIVE_TIME) {
    //   setArmMotor(0.0);
    //   setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    //   setDriveMotors(AUTO_DRIVE_SPEED, 0.0);
    // } else {
    //   setArmMotor(0.0);
    //   setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    //   setDriveMotors(0.0, 0.0);
    // }
  }

  /**
   * Used to remember the last game piece picked up to apply some holding power.
   */
  static final int CONE = 1;
  static final int CUBE = 2;
  static final int NOTHING = 3;
  int lastGamePiece;

  @Override
  public void teleopInit() {
    // Drivetrain.driveLeftSpark.setIdleMode(IdleMode.kCoast);
    // Drivetrain.driveLeftVictor.setNeutralMode(NeutralMode.Coast);
    // Drivetrain.driveRightSpark.setIdleMode(IdleMode.kCoast);
    // Drivetrain.driveRightVictor.setNeutralMode(NeutralMode.Coast);

    lastGamePiece = NOTHING;
  }

  @Override
  public void teleopPeriodic() {
    // double armPower;
    if (j.getLeftBumper()) {
      // lower the arm
      Arm.lowerArm();
    } else if (j.getLeftTriggerAxis()>.9) {
      // raise the arm
      Arm.raiseArm();
    } else {
      // do nothing and let it sit where it is
      Arm.setArmMotor(0.0);
    }
    
    // setArmMotor(0.75*j.getRawAxis(2));

    
    if (j.getRightTriggerAxis()>0.9) {
      // cube in or cone out
      Arm.cubeIn();
      lastGamePiece = CUBE;
    } else if (j.getRawButton(6)){
      // cone in or cube out
      Arm.cubeOut();
      lastGamePiece = CONE;
    } else if (lastGamePiece == CUBE) {
      Arm.holdCube();
    } else if (lastGamePiece == CONE) {
      Arm.holdCone();
    } else {
      Arm.setIntakeMotor(0, 0);
    }
    // setIntakeMotor(intakePower, intakeAmps);

    /*
     * Negative signs here because the values from the analog sticks are backwards
     * from what we want. Forward returns a negative when we want it positive.
     */
    // Drivetrain.setDriveMotors(-j.getRawAxis(1), -j.getRawAxis(4));
    Drivetrain.setDriveMotors(-j.getLeftY(), -j.getRightY());

  }
}
