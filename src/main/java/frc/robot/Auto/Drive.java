package frc.robot.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Drivetrain;

public class Drive extends CommandBase {
    Drivetrain dt = new Drivetrain();
    double time;
    Timer timer = new Timer();

    public Drive(Drivetrain dt, double time) {
        this.dt = dt;
        this.time = time;

        addRequirements(dt);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        dt.setDriveMotors(0.4, 0.6);
    }

    @Override
    public void end(boolean interrupted) {
        dt.setDriveMotors(0, 0);
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= time;
    }

}
