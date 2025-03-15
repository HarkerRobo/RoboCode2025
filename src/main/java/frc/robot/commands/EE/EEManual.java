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
        runTusk();
        runMain();
    }

    private void runTusk() {
        if (RobotContainer.getInstance().getDriver().getRightDPadState()) {
            EndEffector.getInstance().setTuskPower(0.05);
        }
        else if (RobotContainer.getInstance().getDriver().getLeftDPadState())
        {
            EndEffector.getInstance().setTuskPower(-0.05);
        }
        else {
            EndEffector.getInstance().setTuskPower(0);
        }
    }

    private void runMain() {
        if (EndEffector.getInstance().isBackTriggered() && !EndEffector.getInstance().isFrontTriggered())
        {
            EndEffector.getInstance().setMainSpeed(Constants.EndEffector.INTAKE_CORAL_SPEED);
        }
        else if (!EndEffector.getInstance().isBackTriggered() && EndEffector.getInstance().isFrontTriggered())
        {
            EndEffector.getInstance().setMainSpeed(Constants.EndEffector.EJECT_SPEED);
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


    public boolean isFinished ()
    {
        return false;
    }

    public void end (boolean interrupted)
    {
        EndEffector.getInstance().setMainSpeed(0);
    }
}
