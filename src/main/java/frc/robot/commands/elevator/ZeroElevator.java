package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ZeroElevator extends Command {
    public ZeroElevator() {
        addRequirements(Elevator.getInstance());
    }

    public void execute() {
        Elevator.getInstance().setElevatorPower(Constants.Elevator.ZERO_SPEED);
    }

    public boolean isFinished() {
        return Elevator.getInstance().isStalling();
    }

    public void end(boolean interrupted) {
        Elevator.getInstance().setVoltage(0);
        Elevator.getInstance().setSensorPosition(0);
    }

}