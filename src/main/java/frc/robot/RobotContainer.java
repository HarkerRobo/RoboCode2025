// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Function;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.EE.IntakeAlgae;
import frc.robot.commands.EE.EEManual;
import frc.robot.commands.EE.Score;
import frc.robot.commands.EE.TuskMoveToPosition;
import frc.robot.commands.EE.ZeroTusk;
import frc.robot.commands.climb.ClimbManual;
import frc.robot.commands.drivetrain.DriveToPoseCommand;
import frc.robot.commands.elevator.ElevatorManual;
import frc.robot.commands.elevator.MoveToPosition;
import frc.robot.commands.elevator.ZeroElevator;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.swerve.Modules;
import harkerrobolib.joysticks.HSXboxController;

public class RobotContainer {
    public enum AlignDirection
    {
        Left,
        Right,
        Algae,
        LeftBarge,
        MidBarge,
        RightBarge
    }
    private static RobotContainer instance = RobotContainer.getInstance();

    private double MaxSpeed = Constants.Swerve.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second // max angular velocity

    private double MaxSpeedSlow = MaxSpeed * 0.3;
    private double MaxAngularRateSlow = MaxAngularRate * 0.6;

    private AlignDirection direction = AlignDirection.Left;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
            // .withMaximumAcceleration(MaxSpeed * 0.5)
            // .withMaximumAngularAcceleration(MaxAngularRate * 0.5);
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry();

    private final HSXboxController driver = new HSXboxController(0);
    private final HSXboxController operator = new HSXboxController(1);

    public final Drivetrain drivetrain = Modules.createDrivetrain();
    private final Elevator elevator = Elevator.getInstance();
    private final EndEffector endEffector = EndEffector.getInstance();
    private final Climb climb = Climb.getInstance();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("ZeroElevator", new ZeroElevator());
        NamedCommands.registerCommand("ZeroTusk", new ZeroTusk());
        NamedCommands.registerCommand("ElevatorL2",
                new MoveToPosition(Constants.Elevator.CORAL_HEIGHTS[1]));
        NamedCommands.registerCommand("ElevatorL3",
                new MoveToPosition(Constants.Elevator.CORAL_HEIGHTS[2]));
        NamedCommands.registerCommand("ElevatorL4",
                new MoveToPosition(Constants.Elevator.CORAL_HEIGHTS[3]));
        NamedCommands.registerCommand("Score", new Score());
        NamedCommands.registerCommand("ZeroElevatorFast", new MoveToPosition(0));

        autoChooser = AutoBuilder.buildAutoChooser("auton1");
        SmartDashboard.putData("Auton Chooser", autoChooser);

        SignalLogger.start();

        configureBindings();
    }

    private void configureBindings() {


        elevator.setDefaultCommand(new ElevatorManual());

        endEffector.setDefaultCommand(new EEManual());

        climb.setDefaultCommand(new ClimbManual());

        // -- old commands before we changed them --
        // reset the field-centric heading on button b press
        // driver.b().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
        // .andThen(drivetrain.runOnce(() -> drivetrain.resetPose(new Pose2d(new Translation2d(3.20992500, 4.03309382), new Rotation2d(0))))));
        // // driver.rightBumper().onTrue(new Score()
        // //         .andThen(new MoveToPosition(0)
        // //                 .andThen(new ZeroElevator())));

        // // driver.rightTrigger().whileTrue(new DriveToPoseCommand(drivetrain));

        // // driver.x().onTrue(new ScoreManual().raceWith(new WaitCommand(0.5))
        // //         .andThen(new ScoreManual().alongWith(elevator.run( () -> elevator.setVoltage(1.2)))).raceWith(new WaitCommand(2))
        // //         .andThen(new ZeroElevator()));

        // // driver.y().onTrue(new IntakeCoralActive());

        // // driver.leftBumper().onTrue(new IntakeAlgae());

        // // operator.x().onTrue(new MoveToPosition(0).andThen(new ZeroElevator()));
        // // operator.y().onTrue(new MoveToPosition(Constants.Elevator.CORAL_HEIGHTS[3]));
        // // operator.b().onTrue(new MoveToPosition(Constants.Elevator.CORAL_HEIGHTS[2]));
        // // operator.a().onTrue(new MoveToPosition(Constants.Elevator.CORAL_HEIGHTS[1]));

        // // operator.leftBumper().onTrue(new MoveToPosition(Constants.Elevator.ALGAE_HEIGHTS[0]));
        // // operator.rightBumper().onTrue(new MoveToPosition(Constants.Elevator.ALGAE_HEIGHTS[1]));

        // // // operator.getLeftDPad().whileTrue(new DriveToPoseCommand(drivetrain, "Left"));
        // // // operator.getUpDPad().whileTrue(new DriveToPoseCommand(drivetrain, "Algae"));
        // // // operator.getRightDPad().whileTrue(new DriveToPoseCommand(drivetrain, "Right"));



        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        drivetrain.registerTelemetry(logger::telemeterizeDrive);
        logger.telemeterize(elevator, endEffector);
        updateTelemetry();

                // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> {
                    
                    boolean slow = driver.getLeftTriggerAxis() > 0.5;
                    return drive.withVelocityX(-driver.getLeftY() * (slow ? MaxSpeedSlow : MaxSpeed)) // Drive forward with negative Y (forward)
                                .withVelocityY(-driver.getLeftX() * (slow ? MaxSpeedSlow : MaxSpeed)) // Drive left with negative X (left)
                                .withRotationalRate(-driver.getRightX()-driver.getRightX() * (slow ? MaxAngularRateSlow : MaxAngularRate)) // Drive counterclockwise with negative X (left)
                                .withDeadband(slow ? MaxSpeedSlow * 0.03 : MaxSpeed * 0.05);
                }
        ));

        Function<RobotContainer.AlignDirection, Command> setDirectionFactory = ((AlignDirection direction) ->
        new Command() {
            public void execute () {System.out.println(direction); RobotContainer.getInstance().setAlignDirection(direction);}
            public boolean isFinished () {return true;}});

        Command alignLeft = setDirectionFactory.apply(AlignDirection.Left);
        Command alignAlgae = setDirectionFactory.apply(AlignDirection.Algae);
        Command alignRight = setDirectionFactory.apply(AlignDirection.Right);

        operator.getLeftDPad().onTrue(alignLeft); 
        driver.button(7).onTrue(alignLeft);
        // driver left/right bottom button
        
        operator.getUpDPad().onTrue(alignAlgae);
  
        operator.getRightDPad().onTrue(alignRight);
        driver.button(8).onTrue(alignRight);

        configureDriverBindings();
        configureOperatorBindings();

    }

    public void configureDriverBindings ()
    {
        driver.leftBumper().onTrue(endEffector.runOnce(() -> endEffector.setPassive(false))
            .andThen(new MoveToPosition(0))
            .andThen(new TuskMoveToPosition(Constants.EndEffector.GROUND_TUSK_POSITION))
            .andThen(new IntakeAlgae()));
        // Score, wait 1s, zero ET
        driver.rightBumper().onTrue(new Score().withTimeout(1).andThen(new ZeroTusk()).andThen(new MoveToPosition(0)).andThen(new ZeroElevator()));

        driver.rightTrigger().whileTrue(new DriveToPoseCommand(drivetrain));

        driver.b().onTrue(new Score().withTimeout(1)); // Eject (AKA Score)

        driver.x().onTrue(endEffector.runOnce(() -> endEffector.togglePassive()));


    }

    private void configureOperatorBindings ()
    {

        // Levels when left bumper is not pressed

        // L1
        operator.x().and(()->!operator.leftBumper().getAsBoolean())
            .onTrue(new MoveToPosition(Constants.Elevator.CORAL_HEIGHTS[0])
            );

        // L4
        operator.y().and(()->!operator.leftBumper().getAsBoolean())
            .onTrue(new MoveToPosition(Constants.Elevator.CORAL_HEIGHTS[3])
            );
        
        // L3
        operator.b().and(()->!operator.leftBumper().getAsBoolean())
            .onTrue(new MoveToPosition(Constants.Elevator.CORAL_HEIGHTS[2])
            );
        
        // L2
        operator.a().and(()->!operator.leftBumper().getAsBoolean())
            .onTrue(new MoveToPosition(Constants.Elevator.CORAL_HEIGHTS[1])
            );


        // Levels when left bumper is pressed

        // Processor
        operator.x().and(()->operator.leftBumper().getAsBoolean())
            .onTrue(new MoveToPosition(Constants.Elevator.ALGAE_HEIGHTS[0])
            .alongWith(new TuskMoveToPosition(Constants.EndEffector.PROCESSOR_TUSK_POSITION)));

        // Algae Low
        operator.a().and(()->operator.leftBumper().getAsBoolean())
            .onTrue(endEffector.runOnce(() -> endEffector.setPassive(false))
            .andThen(new MoveToPosition(Constants.Elevator.ALGAE_HEIGHTS[1]))
            .andThen(new TuskMoveToPosition(Constants.EndEffector.REEF_TUSK_POSITION))
            .andThen(new IntakeAlgae())); // add zero elevator and tusk

        // Algae High
        operator.b().and(()->operator.leftBumper().getAsBoolean())
            .onTrue(endEffector.runOnce(() -> endEffector.setPassive(false))
            .andThen(new MoveToPosition(Constants.Elevator.ALGAE_HEIGHTS[2]))
            .andThen(new TuskMoveToPosition(Constants.EndEffector.REEF_TUSK_POSITION))
            .andThen(new IntakeAlgae())); // add zero elevator and tusk
        
        // Barge
        operator.y().and(()->operator.leftBumper().getAsBoolean())
            .onTrue(new TuskMoveToPosition(Constants.EndEffector.REEF_TUSK_POSITION)
            .andThen(new MoveToPosition(Constants.Elevator.ALGAE_HEIGHTS[3])
            .alongWith(new TuskMoveToPosition(Constants.EndEffector.BARGE_TUSK_POSITION))));
        
        // Zero ET
        operator.button(8).onTrue(new ZeroElevator().andThen(new ZeroTusk()));

        // Zero DT
        operator.button(7).onTrue(
            drivetrain.runOnce(() -> {System.out.println("home"); drivetrain.seedFieldCentric();})
            .andThen(drivetrain.runOnce(() -> drivetrain.resetPose(
                (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? 
                FlippingUtil.flipFieldPose(new Pose2d(new Translation2d(3.20992500, 4.03309382), new Rotation2d(0))) : 
                new Pose2d(new Translation2d(3.20992500, 4.03309382), new Rotation2d(0))))));
        
    }

    public void updateTelemetry() {
        logger.telemeterize(elevator, endEffector);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public HSXboxController getOperator() {
        return operator;
    }

    public HSXboxController getDriver() {
        return driver;
    }

    public AlignDirection getAlignDirection ()
    {
        return direction;
    }

    public void setAlignDirection (AlignDirection direction)
    {
        this.direction = direction;
    }

    public static RobotContainer getInstance() {
        if (instance == null)
            instance = new RobotContainer();
        return instance;
    }
}