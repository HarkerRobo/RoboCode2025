package frc.robot.commands.EE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;
import harkerrobolib.util.MathUtil;

public class TuskMoveToPositionSimple extends Command {

    private double position;
    public TuskMoveToPositionSimple(double position) {
        addRequirements(EndEffector.getInstance());
        this.position = position;
    }

    public void execute() {
        if (EndEffector.getInstance().getTuskPosition() < position) {
            EndEffector.getInstance().setTuskVoltage(Constants.EndEffector.kG + 3.0);
        }
        else if (EndEffector.getInstance().getTuskPosition() > position)
        {
            EndEffector.getInstance().setTuskVoltage(Constants.EndEffector.kG - 3.0);
        }


        if (EndEffector.getInstance().algaeIn())
        {
            EndEffector.getInstance().setMainSpeed(Constants.EndEffector.ALGAE_HOLD_SPEED);
        }
    }

    public boolean isFinished() {
        return MathUtil.compareSetpoint(EndEffector.getInstance().getTuskPosition(), position, Constants.EndEffector.TUSK_MAX_ERROR);
    }

    @Override
    public void end(boolean interrupted) {
        EndEffector.getInstance().moveToPosition(EndEffector.getInstance().getTuskPosition());
    }

}