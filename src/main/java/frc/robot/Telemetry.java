package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class Telemetry {

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry() {
        SignalLogger.start();
    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable defaultTable = inst.getTable("TEAM1072");

    /* Elevator State */
    private final NetworkTable elevatorStateTable = defaultTable.getSubTable("ElevatorState");
    private final DoublePublisher elevatorSensorPosition = elevatorStateTable.getDoubleTopic("Elevator Sensor Position").publish();
    private final DoublePublisher elevatorSensorVelocity = elevatorStateTable.getDoubleTopic("Elevator Sensor Velocity").publish();
    private final DoublePublisher elevatorDesiredPosition = elevatorStateTable.getDoubleTopic("Elevator Desired Position").publish();

    /* End Effector State */
    private final NetworkTable endEffectorStateTable = defaultTable.getSubTable("EndEffectorState");
    private final BooleanPublisher frontCanandcolorHit = endEffectorStateTable.getBooleanTopic("EE Front Canandcolor Hit").publish();
    private final BooleanPublisher backCanandcolorHit = endEffectorStateTable.getBooleanTopic("EE Back Canandcolor Hit").publish();
    private final DoublePublisher tuskPosition = endEffectorStateTable.getDoubleTopic("Tusk Position").publish();
    private final DoublePublisher tuskDesiredPosition = endEffectorStateTable.getDoubleTopic("Tusk Desired Position").publish();
    private final DoublePublisher mainCurrent = endEffectorStateTable.getDoubleTopic("Main Current").publish();
    private final BooleanPublisher isMainStalling = endEffectorStateTable.getBooleanTopic("Is Main Stalling").publish();
    private final BooleanPublisher isAlgaeIn = endEffectorStateTable.getBooleanTopic("Is Algae In").publish();
    private final BooleanPublisher isPassive = endEffectorStateTable.getBooleanTopic("Is Passive On").publish();
    
    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = defaultTable.getSubTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Robot Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();

    private final double[] m_poseArray = new double[3];
    private final double[] m_moduleStatesArray = new double[8];
    private final double[] m_moduleTargetsArray = new double[8];

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterizeDrive(SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModulePositions.set(state.ModulePositions);

        /* Also write to log file */
        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        for (int i = 0; i < 4; ++i) {
            m_moduleStatesArray[i*2 + 0] = state.ModuleStates[i].angle.getRadians();
            m_moduleStatesArray[i*2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
            m_moduleTargetsArray[i*2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
        }

        SignalLogger.writeDoubleArray("DriveState/Pose", m_poseArray);
        SignalLogger.writeDoubleArray("DriveState/ModuleStates", m_moduleStatesArray);
    }

    public void telemeterize(Elevator elevator, EndEffector endEffector) {
        elevatorSensorPosition.set(elevator.getPosition());
        elevatorSensorVelocity.set(elevator.getVelocity());
        elevatorDesiredPosition.set(elevator.getDesiredPosition());

        frontCanandcolorHit.set(endEffector.isFrontTriggered());
        backCanandcolorHit.set(endEffector.isBackTriggered());

        tuskPosition.set(endEffector.getTuskPosition());
        tuskDesiredPosition.set(endEffector.getTuskDesiredPosition());
        mainCurrent.set(endEffector.getMainMotorCurrent());
        isMainStalling.set(endEffector.isMainStalling());
        isAlgaeIn.set(endEffector.algaeIn());
        isPassive.set(endEffector.getPassive());
    }
}