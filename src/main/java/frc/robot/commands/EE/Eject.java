package frc.robot.commands.EE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class Eject extends Command {
    public Eject() {
        addRequirements(EndEffector.getInstance());
    }

    public void execute () {
        EndEffector.getInstance().setMainSpeed(Constants.EndEffector.EJECT_SPEED);
    }

    public boolean isFinished () {
        return !EndEffector.getInstance().isFrontTriggered();
    }

    public void end (boolean interrupted) {
        EndEffector.getInstance().setMainSpeed(0);
    }
}