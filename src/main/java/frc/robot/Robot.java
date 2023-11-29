// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.LED;
// import frc.robot.Subsystems.Arm;

public class Robot extends TimedRobot {
  /*
   * Autonomous selection options.
   */

  private static final String kNothingAuto = "do nothing";
  private static final String kCubeAuto = "cube";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // SendableChooser<Command> m_chooser = new SendableChooser<>();
  // private Command m_autonomousCommand;

  // private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // private Drivetrain dt = new Drivetrain();
  // private Arm arm = new Arm();
  private LED led = new LED();
  private Drivetrain drive = new Drivetrain();
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
  public XboxController j = new XboxController(0);
  Trigger test = new JoystickButton(j, Button.kY.value).onTrue(new SequentialCommandGroup(
      // new ParallelDeadlineGroup(
      // new WaitCommand(5),
      // //currently for tank drive
      // new InstantCommand(() -> {
      // Drivetrain.setDriveMotors(-0.2, -0.2);
      // System.out.println("AUTO RUNNING");
      // })
      // // new setDriveMotors(-0.5, -0.5)
      // )

      new InstantCommand(() -> {
        System.out.println("auto running");
      })));

  /**
   * This method is run once when the robot is first started up.
   */
  @Override
  public void robotInit() {
    // m_chooser.setDefaultOption("do nothing", new InstantCommand(()->{};));
    // m_chooser.addOption("cone and mobility", kConeAuto);

    m_chooser.setDefaultOption("do nothing", kNothingAuto);
    m_chooser.addOption("onepiece", kCubeAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // m_chooser.setDefaultOption("One Piece Auto", new OnePieceAuto(dt, arm));

    SmartDashboard.putData("Auto choices", m_chooser);

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
  // SmartDashboard.putNumber("drive forward power (%)", forward);
  // SmartDashboard.putNumber("drive turn power (%)", turn);

  // /*
  // * positive turn = counter clockwise, so the left side goes backwards
  // */
  // double left = forward - turn;
  // double right = forward + turn;

  // SmartDashboard.putNumber("drive left power (%)", left);
  // SmartDashboard.putNumber("drive right power (%)", right);

  // // see note above in robotInit about commenting these out one by one to set
  // // directions.

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

    // led.rainbow();
  }

  double autonomousStartTime;
  double autonomousIntakePower;
  Command m_command;

  @Override
  public void autonomousInit() {

    m_autoSelected = m_chooser.getSelected();

    if (m_autoSelected == kCubeAuto) {
      // Arm.cubeOut();
    }

    autonomousStartTime = Timer.getFPGATimestamp();

    // if (m_autonomousCommand != null) {
    // m_autonomousCommand.schedule();
    // }
    // final Command m_simpleAuto =
    // new OnePieceAuto(dt, arm);
    // new InstantCommand(()->{new OnePieceAuto(dt, arm);});
    // m_command = m_chooser.getSelected();
    // System.out.println("Auto selected: " + m_autoSelected);
    // m_chooser.getSelected().schedule();
    // if (m_autoSelected == kConeAuto) {
    // autonomousIntakePower = INTAKE_OUTPUT_POWER;
    // } else if (m_autoSelected == kCubeAuto) {
    // autonomousIntakePower = -INTAKE_OUTPUT_POWER;
    // }

    // autonomousStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void disabledInit() {
    // led.rainbow();
  }

  @Override
  public void disabledPeriodic() {
    led.rainbow();
  }

  @Override
  public void autonomousPeriodic() {

    if (m_autoSelected == kNothingAuto) {
      // Arm.setArmMotor(0.0);
      // Arm.setIntakeMotor(0.0, 25);
      drive.setDriveMotors(0.0, 0.0);
      return;
    }

    double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;
    final double ARM_EXTEND_TIME_S = 2.0;
    final double AUTO_THROW_TIME_S = 0.375;
    final double AUTO_DRIVE_TIME = 1.0;

    if (timeElapsed < ARM_EXTEND_TIME_S) {
      // Arm.raiseArm();
      // Arm.stopIntake();
      drive.setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S) {
      // Arm.stopArm();
      // Arm.cubeOut();
      drive.setDriveMotors(0.5, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S) {
      // Arm.stopArm();
      // Arm.stopIntake();
      drive.setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + AUTO_DRIVE_TIME) {
      // Arm.stopArm();
      // Arm.stopIntake();
      drive.setDriveMotors(-0.5, 0.0);
    } else {
      // Arm.stopArm();
      // Arm.stopIntake();
      drive.setDriveMotors(0.0, 0.0);
    }

    // System.out.println("AUTO Periodic");
    // if (m_autoSelected == kNothingAuto) {
    // setArmMotor(0.0);
    // setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    // setDriveMotors(0.0, 0.0);
    // return;
    // }

    // double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;

    // if (timeElapsed < ARM_EXTEND_TIME_S) {
    // setArmMotor(ARM_OUTPUT_POWER);
    // setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    // setDriveMotors(0.0, 0.0);
    // } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S) {
    // setArmMotor(0.0);
    // setIntakeMotor(autonomousIntakePower, INTAKE_CURRENT_LIMIT_A);
    // setDriveMotors(0.0, 0.0);
    // } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S +
    // ARM_EXTEND_TIME_S) {
    // setArmMotor(-ARM_OUTPUT_POWER);
    // setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    // setDriveMotors(0.0, 0.0);
    // } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S +
    // ARM_EXTEND_TIME_S + AUTO_DRIVE_TIME) {
    // setArmMotor(0.0);
    // setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    // setDriveMotors(AUTO_DRIVE_SPEED, 0.0);
    // } else {
    // setArmMotor(0.0);
    // setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    // setDriveMotors(0.0, 0.0);
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
      // Arm.lowerArm();
    } else if (j.getLeftTriggerAxis() > .5) {
      // raise the arm
      // Arm.raiseArm();
    } else {
      // do nothing and let it sit where it is
      // Arm.setArmMotor(0.0);
    }

    if (j.getRightTriggerAxis() != 0) {
      // cube in or cone out
      // Arm.cubeIn();
      lastGamePiece = CUBE;
      led.purple();
    } else if (j.getRawButton(6)) {
      // cone in or cube out
      // Arm.cubeOut();
      lastGamePiece = CONE;
      led.yellow();
    } else if (lastGamePiece == CUBE) {
      // Arm.holdCube();
    } else if (lastGamePiece == CONE) {
      // Arm.holdCone();
    } else {
      // Arm.setIntakeMotor(0, 0);
    }

    // if(j.getAButton()){
    // led.rainbow();
    // }

    /******************************************/
    /* Joystic Deadzone */
    /******************************************/

    // Drivetrain.setDriveMotors(j.getLeftY(), j.getRightX());

    if (Math.abs(j.getLeftY()) > 0.1 || Math.abs(j.getRightX()) > 0.1) {
      drive.setDriveMotors(j.getLeftY(), j.getRightX());
    } else {
      drive.setDriveMotors(0, 0);
    }
  }
}
