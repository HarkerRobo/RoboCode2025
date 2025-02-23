package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;

public class Climb extends Command {
    public Climb() {
        addRequirements(frc.robot.subsystems.Climb.getInstance());
    }

    public void execute () {
        frc.robot.subsystems.Climb.getInstance().setSpeed(RobotMap.Climb.CLIMB_VELOCITY);
    }

    public boolean isFinished () {
        return false;
    }

    public void end (boolean interrupted) 
    {
        frc.robot.subsystems.Climb.getInstance().moveToPosition(RobotMap.Climb.RESET_POSITION);
    }
}
