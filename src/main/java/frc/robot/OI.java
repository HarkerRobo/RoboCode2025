package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.EE.Eject;
import frc.robot.commands.EE.IntakeAlgae;
import frc.robot.commands.EE.IntakeCoral;
import frc.robot.commands.EE.Score;
import frc.robot.commands.elevator.DoOperator;
import frc.robot.commands.elevator.MoveToPosition;
import frc.robot.commands.elevator.SetOperatorPosition;
import frc.robot.commands.elevator.ZeroElevator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.util.XboxGamepad;

public class OI {
    private static OI instance;
    private XboxGamepad driver;
    private XboxGamepad operator;

    private OI() {
        driver = new XboxGamepad(RobotMap.OI.DRIVER_ID);
        operator = new XboxGamepad(RobotMap.OI.OPERATOR_ID);

        initBindings();
    }

    public XboxGamepad getDriver() {
        return driver;
    }

    public XboxGamepad getOperator() {
        return operator;
    }

    private void initBindings() {
        // driver.getButtonA().onTrue(new InstantCommand(() -> {
            // Drivetrain.getInstance().toggleRobotCentric();
        // }));

        driver.getButtonB().onTrue(new InstantCommand( () -> Drivetrain.getInstance().setPose(new Pose2d(0, 0, Rotation2d.fromDegrees(90)))));

        // combine into one in the future?
        driver.getButtonX().onTrue(new IntakeAlgae());

        driver.getRightBumper().onTrue(new DoOperator()
                                    .andThen(new Score())
                                    .andThen(new SetOperatorPosition(0))
                                    .andThen(new DoOperator())
                                    .andThen(new ZeroElevator()));

        // IMPORTANT: desired height is from 1-4 while level_heights[i] indexing is from 0-3
        // driver.getRightBumper().whileTrue(new MoveToPosition(RobotMap.Elevator.LEVEL_HEIGHTS[Elevator.getInstance().getDesiredLevel()-1]));

        driver.getButtonY().whileTrue(new ZeroElevator());
        driver.getButtonA().whileTrue(new DoOperator());
        
        operator.getButtonX().whileTrue(new SetOperatorPosition(0).andThen(new ZeroElevator()));
        operator.getButtonY().whileTrue(new SetOperatorPosition(RobotMap.Elevator.LEVEL_HEIGHTS[3]));
        operator.getButtonB().whileTrue(new SetOperatorPosition(RobotMap.Elevator.LEVEL_HEIGHTS[2]));
        operator.getButtonA().onTrue(new SetOperatorPosition(RobotMap.Elevator.LEVEL_HEIGHTS[1]));
    }

    public static OI getInstance() {
        if (instance == null) instance = new OI();
        return instance;
    }
    
}
