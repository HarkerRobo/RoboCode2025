package frc.robot.commands.EE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class Score extends Command {

    public Score () {
        addRequirements(EndEffector.getInstance());
    }

    public void execute () {
        EndEffector.getInstance().setMainSpeed(Constants.EndEffector.OUTTAKE_SPEED);
    }

    public boolean isFinished () {
        return (!EndEffector.getInstance().isFrontTriggered() && !EndEffector.getInstance().isBackTriggered());
    }

    public void end (boolean interrupted) {
        EndEffector.getInstance().setMainSpeed(0);
    }
}