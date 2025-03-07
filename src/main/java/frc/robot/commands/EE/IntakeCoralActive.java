package frc.robot.commands.EE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class IntakeCoralActive extends Command {
    public IntakeCoralActive () {
        addRequirements(EndEffector.getInstance());
    }

    public void execute () {
        if (!EndEffector.getInstance().isBackTriggered() && EndEffector.getInstance().isFrontTriggered())
        {
            EndEffector.getInstance().setSpeed(Constants.EndEffector.EJECT_SPEED);
        }
        else
        {
            EndEffector.getInstance().setSpeed(Constants.EndEffector.INTAKE_CORAL_SPEED);
        }
    }

    public boolean isFinished () {
        return EndEffector.getInstance().isBackTriggered() && EndEffector.getInstance().isFrontTriggered();
    }

    public void end (boolean interrupted) {
        EndEffector.getInstance().setSpeed(0);
    }
}