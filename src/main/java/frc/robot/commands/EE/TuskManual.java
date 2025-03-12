package frc.robot.commands.EE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.EndEffector;

public class TuskManual extends Command {

    public TuskManual() {
        addRequirements(EndEffector.getInstance());
    }

    public void execute () {
        if (RobotContainer.getInstance().getDriver().getRightDPadState()) {
            EndEffector.getInstance().setTuskPower(0.2);;
        }
        else if (RobotContainer.getInstance().getDriver().getLeftDPadState())
        {
            EndEffector.getInstance().setTuskPower(-0.2);
        }
        else {
            EndEffector.getInstance().setTuskPower(0);
        }
    }

    public boolean isFinished() {
        return false;
    }


    public void end (boolean interrupted) 
    {
        EndEffector.getInstance().setTuskPower(0);
    }
}