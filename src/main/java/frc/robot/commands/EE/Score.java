package frc.robot.commands.EE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class Score extends Command {

    public Score () {
        addRequirements(EndEffector.getInstance());
    }

    public void execute () {
        EndEffector.getInstance().setMainSpeed(
            EndEffector.getInstance().algaeIn() ? 
            Constants.EndEffector.ALGAE_SCORE_SPEED : Constants.EndEffector.CORAL_SCORE_SPEED);
    }

    public boolean isFinished () {
        return (!EndEffector.getInstance().isFrontTriggered() && !EndEffector.getInstance().isBackTriggered());
    }

    public void end (boolean interrupted) {
        EndEffector.getInstance().setMainSpeed(0);
        EndEffector.getInstance().setPassive(true);
        EndEffector.getInstance().setAlgaeIn(false);
    }
}