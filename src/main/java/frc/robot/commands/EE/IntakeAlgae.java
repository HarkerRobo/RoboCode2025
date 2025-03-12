package frc.robot.commands.EE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffector;

public class IntakeAlgae extends Command {
    public IntakeAlgae () {
        addRequirements(EndEffector.getInstance());
    }

    public void execute () {
        EndEffector.getInstance().setMainSpeed(Constants.EndEffector.INTAKE_ALGAE_SPEED);
    }

    public boolean isFinished () {
        return EndEffector.getInstance().isFrontTriggered();
    }

    public void end (boolean interrupted) {
        EndEffector.getInstance().setMainSpeed(Constants.EndEffector.ALGAE_HOLD_SPEED);
    }
}