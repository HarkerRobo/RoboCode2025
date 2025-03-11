package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.MathUtil;
import frc.robot.subsystems.Elevator;
// import harkerrobolib.util.MathUtil;

public class MoveToPosition extends Command {

    public double height;
    public MoveToPosition(double height) {
        addRequirements(Elevator.getInstance());
        this.height = height;
    }

    public void execute() {
        Elevator.getInstance().moveToPosition(height);
    }

    public boolean isFinished() {
        return MathUtil.compareSetpoint(Elevator.getInstance().getPosition(), height, Constants.Elevator.MAX_ERROR);
    }

    @Override
    public void end(boolean interrupted) {
        // Elevator.getInstance().setElevatorPower(0);
    }

}