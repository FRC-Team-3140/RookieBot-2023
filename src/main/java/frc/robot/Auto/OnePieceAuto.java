package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Drivetrain;

public class OnePieceAuto extends CommandBase {

    Drivetrain dt;
    Arm arm;

    public OnePieceAuto(Drivetrain dt, Arm arm) {

        this.dt = dt;
        this.arm = arm;
        addRequirements(dt, arm);
    }

    @Override
    public void initialize() {
        new SequentialCommandGroup(
                // new ParallelDeadlineGroup(
                // new WaitCommand(5),
                // //currently for tank drive
                // new InstantCommand(() -> {
                // Drivetrain.setDriveMotors(-0.2, -0.2);
                // System.out.println("AUTO RUNNING");
                // })
                // // new setDriveMotors(-0.5, -0.5)
                // )

                new Drive(dt, 2),
                new InstantCommand(() -> {
                    System.out.println("auto running");
                })).schedule();
    }

}
