package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import harkerrobolib.util.MathUtil;

public class ElevatorManual extends Command {

    public ElevatorManual() {
        addRequirements(Elevator.getInstance());
    }

    @Override
    public void execute() {
        if (Math.abs(RobotContainer.getInstance().getOperator().getLeftY()) > 0.2) {
            Elevator.getInstance().moveToPosition(Elevator.getInstance().getPosition()
                + MathUtil.mapJoystickOutput(RobotContainer.getInstance().getOperator().getLeftY(), 0.2)*0.2);
        }
        else
        {
            Elevator.getInstance().moveToPosition(Elevator.getInstance().getPosition());
        }
    }

    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Elevator.getInstance().setElevatorPower(0);
    }
}