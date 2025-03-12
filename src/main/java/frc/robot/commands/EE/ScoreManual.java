package frc.robot.commands.EE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class ScoreManual extends Command {

    public ScoreManual () {
        addRequirements(EndEffector.getInstance());
    }

    public void execute () {
        EndEffector.getInstance().setMainSpeed(Constants.EndEffector.OUTTAKE_SPEED_L1);
    }

    public boolean isFinished () {
        return false;
    }

    public void end (boolean interrupted) {
        EndEffector.getInstance().setMainSpeed(0);
    }
}