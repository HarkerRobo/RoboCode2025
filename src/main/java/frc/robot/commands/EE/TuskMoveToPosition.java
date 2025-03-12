package frc.robot.commands.EE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;
import harkerrobolib.util.MathUtil;

public class TuskMoveToPosition extends Command {

    public double position;
    public TuskMoveToPosition(double position) {
        addRequirements(EndEffector.getInstance());
        this.position = position;
    }

    public void execute() {
        EndEffector.getInstance().moveToPosition(position);
    }

    public boolean isFinished() {
        return MathUtil.compareSetpoint(EndEffector.getInstance().getTuskPosition(), position, Constants.EndEffector.TUSK_MAX_ERROR);
    }

    @Override
    public void end(boolean interrupted) {
        EndEffector.getInstance().setTuskVoltage(0);
    }

}