package frc.robot.commands.EE;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class Score extends Command {

    Timer timer;
    public Score () {
        timer = new Timer();
        timer.start();
        addRequirements(EndEffector.getInstance());
    }

    public void initialize ()
    {
        timer.reset();
    }

    public void execute () {
        if (EndEffector.getInstance().algaeIn())
        {
            EndEffector.getInstance().setMainSpeed(Constants.EndEffector.ALGAE_SCORE_SPEED);
            EndEffector.getInstance().moveToPosition(Constants.EndEffector.REEF_TUSK_POSITION);
        }
        else
        {
            EndEffector.getInstance().setMainSpeed(Constants.EndEffector.CORAL_SCORE_SPEED);
        }
    }

    public boolean isFinished () {
        return EndEffector.getInstance().algaeIn() ? timer.hasElapsed(0.5) : (!EndEffector.getInstance().isFrontTriggered() && !EndEffector.getInstance().isBackTriggered());
    }

    public void end (boolean interrupted) {
        EndEffector.getInstance().setMainSpeed(0);
        EndEffector.getInstance().setPassive(true);
        EndEffector.getInstance().setAlgaeIn(false);
    }
}