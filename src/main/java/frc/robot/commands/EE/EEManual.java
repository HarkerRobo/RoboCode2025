package frc.robot.commands.EE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.EndEffector;

public class EEManual extends Command {
    public EEManual () {
        addRequirements(EndEffector.getInstance());
    }

    public void execute () {
        if (EndEffector.getInstance().isMainStalling())
        {
            EndEffector.getInstance().setStalling(true);
        }
        
        runTusk();
        runMain();
    }

    private void runTusk() {
        if (RobotContainer.getInstance().getDriver().getRightDPadState()) {
            EndEffector.getInstance().setTuskPower(0.1);
        }
        else if (RobotContainer.getInstance().getDriver().getLeftDPadState())
        {
            EndEffector.getInstance().setTuskPower(-0.1);
        }
        else {
            EndEffector.getInstance().setTuskPower(0);
        }
    }

    private void runMain() {
        if (EndEffector.getInstance().isCoral())
        {
            if (EndEffector.getInstance().isBackTriggered() && !EndEffector.getInstance().isFrontTriggered())
            {
                EndEffector.getInstance().setMainSpeed(Constants.EndEffector.INTAKE_CORAL_SPEED);
            }
            else if (!EndEffector.getInstance().isBackTriggered() && EndEffector.getInstance().isFrontTriggered())
            {
                EndEffector.getInstance().setMainSpeed(Constants.EndEffector.REVERSE_INTAKE_SPEED);
            }
            else if (EndEffector.getInstance().isBackTriggered() && EndEffector.getInstance().isFrontTriggered())
            {
                EndEffector.getInstance().setMainSpeed( 0);
            }
            else
            {
                EndEffector.getInstance().setMainSpeed(
                EndEffector.getInstance().getPassive() ? 
                Constants.EndEffector.INTAKE_CORAL_SLOW_SPEED : 0);
            }
        }
        else
        {
            if (!EndEffector.getInstance().hasStalled())
            {
                EndEffector.getInstance().setMainSpeed(Constants.EndEffector.INTAKE_ALGAE_SPEED);
            }
            else 
            {
                EndEffector.getInstance().setMainSpeed(Constants.EndEffector.ALGAE_HOLD_SPEED);
            }
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
