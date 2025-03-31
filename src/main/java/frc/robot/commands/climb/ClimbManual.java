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
            Climb.getInstance().setVelocity(1.5);
        }
        else if (RobotContainer.getInstance().getDriver().getDownDPadState() || RobotContainer.getInstance().getDriver().leftBumper().getAsBoolean())
        {
            Climb.getInstance().setVelocity(-2);
        }
        else {
            Climb.getInstance().setDutyCycle(0);
        }
    }

    public boolean isFinished() {
        return false;
    }


    public void end (boolean interrupted) 
    {
        Climb.getInstance().setDutyCycle(0);
    }
}