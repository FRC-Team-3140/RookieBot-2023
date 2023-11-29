package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  static TalonSRX arm = new TalonSRX(5);
  static TalonSRX arm2 = new TalonSRX(7);
  static VictorSPX intake = new VictorSPX(6);
  static final int ARM_CURRENT_LIMIT_A = 25;
  static final double ARM_OUTPUT_POWER = 0.5;
  static final int INTAKE_CURRENT_LIMIT_A = 15;
  static final int INTAKE_HOLD_CURRENT_LIMIT_A = 2;
  static final double INTAKE_OUTPUT_POWER = -0.9;
  static final double INTAKE_HOLD_POWER = -0.07;
  static final double ARM_EXTEND_TIME_S = 2.0;
  static final double AUTO_THROW_TIME_S = 0.375;
  static final double AUTO_DRIVE_TIME = 6.0;
  static final double AUTO_DRIVE_SPEED = -0.25;

  public Arm() {
    arm.setInverted(false);
    arm2.setInverted(true);
    arm.setNeutralMode(NeutralMode.Brake);
    arm.setNeutralMode(NeutralMode.Brake);

    // arm.setSmartCurrentLimit(ARM_CURRENT_LIMIT_A);
    intake.setInverted(false);
    intake.setNeutralMode(NeutralMode.Brake);

    // intake.setIdleMode(IdleMode.kBrake);
  }

  public static void setArmMotor(double percent) {
    arm.set(TalonSRXControlMode.PercentOutput, percent);
    arm2.set(TalonSRXControlMode.PercentOutput, percent);

    SmartDashboard.putNumber("arm power (%)", percent);
    SmartDashboard.putNumber("arm motor current (amps)", arm.getStatorCurrent());
    // SmartDashboard.putNumber("arm motor temperature (C)", arm.get);
  }

  public static void setIntakeMotor(double percent, int amps) {
    intake.set(VictorSPXControlMode.PercentOutput, percent);
    // intake.set(percent);
    // intake.setSmartCurrentLimit(amps);
    SmartDashboard.putNumber("intake power (%)", percent);
    // SmartDashboard.putNumber("intake motor current (amps)", intake.get());
    // SmartDashboard.putNumber("intake motor temperature (C)",
    // intake.getMotorTemperature());
  }

  // double armPower;
  public static void lowerArm() {
    setArmMotor(-ARM_OUTPUT_POWER);
  }

  public static void raiseArm() {
    setArmMotor(ARM_OUTPUT_POWER + 0.1); // arm_output_power is 0.4
  }

  public static void cubeIn() {
    setIntakeMotor(INTAKE_OUTPUT_POWER, INTAKE_CURRENT_LIMIT_A);
  }

  public static void cubeOut() {
    setIntakeMotor(-INTAKE_OUTPUT_POWER, INTAKE_CURRENT_LIMIT_A);
  }

  public static void holdCube() {
    setIntakeMotor(INTAKE_HOLD_POWER, INTAKE_HOLD_CURRENT_LIMIT_A);
  }

  public static void holdCone() {
    setIntakeMotor(-INTAKE_HOLD_POWER, INTAKE_HOLD_CURRENT_LIMIT_A);
    ;
  }

  public static void stopIntake() {
    setIntakeMotor(0, 0);
  }

  public static void stopArm() {
    setArmMotor(0);
  }
}
