package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climb;

public class ClimbManual extends Command {

    public ClimbManual() {
        addRequirements(Climb.getInstance());
    }

    public void execute () {
        if (RobotContainer.getInstance().getDriver().getUpDPadState()) {
            Climb.getInstance().moveToPosition(Climb.getInstance().getPosition()+0.2);
        }
        else if (RobotContainer.getInstance().getDriver().getDownDPadState())
        {
            Climb.getInstance().moveToPosition(Climb.getInstance().getPosition()-0.2);
        }

        else
        {
            Climb.getInstance().moveToPosition(Climb.getInstance().getPosition());
        }
    }

    public boolean isFinished() {
        return false;
    }


    public void end (boolean interrupted) 
    {
        Climb.getInstance().setVelocity(0);
    }
}