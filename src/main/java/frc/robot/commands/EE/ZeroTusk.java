package frc.robot.commands.EE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class ZeroTusk extends Command {
    public ZeroTusk() {
        addRequirements(EndEffector.getInstance());
    }

    public void execute() {
        EndEffector.getInstance().setTuskPower(Constants.EndEffector.TUSK_ZERO_SPEED);
    }

    public boolean isFinished() {
        return EndEffector.getInstance().isStalling();
    }

    public void end(boolean interrupted) {
        EndEffector.getInstance().setTuskVoltage(0);
        EndEffector.getInstance().setTuskSensorPosition(0);
    }

}