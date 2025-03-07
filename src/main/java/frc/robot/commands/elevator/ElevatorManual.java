package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;

public class ElevatorManual extends Command {

    public ElevatorManual() {
        addRequirements(Elevator.getInstance());
    }

    @Override
    public void execute() {
        if (RobotContainer.getInstance().getOperator().getLeftY() < -0.5) {
            Elevator.getInstance().moveToPosition(Elevator.getInstance().getPosition()+0.2);
        }
        else if (RobotContainer.getInstance().getOperator().getLeftY() > 0.5)
        {
            Elevator.getInstance().moveToPosition(Elevator.getInstance().getPosition()-0.2);
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