package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
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


// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
// import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drivetrain {
    public static CANSparkMax driveLeftSpark = new CANSparkMax(1, MotorType.kBrushed);
    public static CANSparkMax driveRightSpark = new CANSparkMax(2, MotorType.kBrushed);
    public static VictorSPX driveLeftVictor = new VictorSPX(3);
    public static VictorSPX driveRightVictor = new VictorSPX(4);
    
    // driveLeftSpark.setIdleMode(IdleMode.kCoast);
    // driveLeftVictor.setNeutralMode(NeutralMode.Coast);
    // driveRightSpark.setIdleMode(IdleMode.kCoast);
    // driveRightVictor.setNeutralMode(NeutralMode.Coast);

    public Drivetrain(){
        driveLeftSpark.setInverted(false);
        driveLeftVictor.setInverted(false);
        driveRightSpark.setInverted(false);
        driveRightVictor.setInverted(false);
    }

    public static void setDriveMotors(double forward, double turn) {
        driveLeftSpark.setIdleMode(IdleMode.kCoast);
        driveLeftVictor.setNeutralMode(NeutralMode.Coast);
        driveRightSpark.setIdleMode(IdleMode.kCoast);
        driveRightVictor.setNeutralMode(NeutralMode.Coast);

        SmartDashboard.putNumber("drive forward power (%)", forward);
        SmartDashboard.putNumber("drive turn power (%)", turn);

        /*
        * positive turn = counter clockwise, so the left side goes backwards
        */
        double left = forward - turn;
        double right = forward + turn;

        SmartDashboard.putNumber("drive left power (%)", left);
        SmartDashboard.putNumber("drive right power (%)", right);

        // see note above in robotInit about commenting these out one by one to set
        // directions.
        // driveLeftSpark.set(left);
        // driveLeftVictor.set(ControlMode.PercentOutput, left);
        // driveRightSpark.set(right);
        // driveRightVictor.set(ControlMode.PercentOutput, right);

        driveLeftSpark.set(forward);
        driveLeftVictor.set(ControlMode.PercentOutput, forward);
        driveRightSpark.set(turn);
        driveRightVictor.set(ControlMode.PercentOutput, turn);
  }










    //   // The robot's drive
//   private final DifferentialDrive dt = new DifferentialDrive(leftSideMotors, rightSideMotors);

//   /** Creates a new DriveSubsystem. */
//   public Drivetrain() {
//     // We need to invert one side of the drivetrain so that positive voltages
//     // result in both sides moving forward. Depending on how your robot's
//     // gearbox is constructed, you might have to invert the left side instead.
//     leftEncoder.setPositionConversionFactor(kDriveTrainEncoderMetersPerPulse);
//     rightEncoder.setPositionConversionFactor(kDriveTrainEncoderMetersPerPulse);
//     leftEncoder.setVelocityConversionFactor(kDriveTrainEncoderLinearMetersPerSecondPerRPM);
//     rightEncoder.setVelocityConversionFactor(kDriveTrainEncoderLinearMetersPerSecondPerRPM);
//     rightMotor1.setSmartCurrentLimit(60);
//     rightMotor2.setSmartCurrentLimit(60);
//     rightMotor3.setSmartCurrentLimit(60);
//     leftMotor1.setSmartCurrentLimit(60);
//     leftMotor2.setSmartCurrentLimit(60);
//     leftMotor3.setSmartCurrentLimit(60);
//   }

//   /**
//    * Arcade drive method for differential drive platform.
//    *
//    * @param speed The robot's speed along the foward/reverse axis [-1.0..1.0]. Forward is positive.
//    * @param rotation The robot's rotation rate around the Z/vertical axis [-1.0..1.0]. Clockwise is
//    *     positive.
//    */
//   public void arcadeDrive(double speed, double rotation) {
//     dt.arcadeDrive(speed, rotation, false);
//     //We do not need to square inputs because we cube the inputs
//     // with SmoothXboxController's getSmooth____() classes so the
//     //the last argument to arcadeDrive is false
  }