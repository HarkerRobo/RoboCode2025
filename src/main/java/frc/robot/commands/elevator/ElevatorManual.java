package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
// import harkerrobolib.util.MathUtil;

public class ElevatorManual extends Command {

    private boolean holdPos;

    public ElevatorManual() {
        addRequirements(Elevator.getInstance());
        holdPos = false;
    }

    @Override
    public void execute() {
        if (Math.abs(-RobotContainer.getInstance().getOperator().getLeftY()) > 0.2) {
            Elevator.getInstance().setVoltage(Constants.Elevator.kG + MathUtil.mapJoystickOutput(-RobotContainer.getInstance().getOperator().getLeftY(), 0.2));
            holdPos = true;
        }
        else if (holdPos)
        {
            Elevator.getInstance().moveToPosition(Elevator.getInstance().getPosition());
            holdPos = false;
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