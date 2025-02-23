package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Climb;

public class ResetClimb extends Command {
    public ResetClimb() {
        addRequirements(Climb.getInstance());
    }

    public void execute () {
        Climb.getInstance().moveToPosition(RobotMap.Climb.RESET_POSITION);
    }

    public boolean isFinished () {
        return Math.abs(Climb.getInstance().getPosition() - RobotMap.Climb.RESET_POSITION) < 0.1;
    }

    public void end (boolean interrupted) {
        Climb.getInstance().setSpeed(0);
    }
}
