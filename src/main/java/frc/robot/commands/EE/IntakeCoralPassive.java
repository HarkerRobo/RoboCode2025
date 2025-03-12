package frc.robot.commands.EE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class IntakeCoralPassive extends Command {
    public IntakeCoralPassive () {
        addRequirements(EndEffector.getInstance());
    }

    public void execute () {
        if (EndEffector.getInstance().isBackTriggered() && !EndEffector.getInstance().isFrontTriggered())
        {
            EndEffector.getInstance().setMainSpeed(Constants.EndEffector.INTAKE_CORAL_SPEED);
        }
        else if (!EndEffector.getInstance().isBackTriggered() && EndEffector.getInstance().isFrontTriggered())
        {
            EndEffector.getInstance().setMainSpeed(Constants.EndEffector.EJECT_SPEED);
        }
        else
        {
            EndEffector.getInstance().setMainSpeed(0);
        }
    }

    public boolean isFinished ()
    {
        return false;
    }

    public void end (boolean interrupted)
    {
        EndEffector.getInstance().setMainSpeed(0);
    }
}
