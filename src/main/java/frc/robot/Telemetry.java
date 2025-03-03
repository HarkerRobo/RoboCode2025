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
import edu.wpi.first.networktables.StringPublisher;
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
    
    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = defaultTable.getSubTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Robot Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
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
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);

        /* Also write to log file */
        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        for (int i = 0; i < 4; ++i) {
            m_moduleStatesArray[i*2 + 0] = state.ModuleStates[i].angle.getRadians();
            m_moduleStatesArray[i*2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
            m_moduleTargetsArray[i*2 + 0] = state.ModuleTargets[i].angle.getRadians();
            m_moduleTargetsArray[i*2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
        }

        SignalLogger.writeDoubleArray("DriveState/Pose", m_poseArray);
        SignalLogger.writeDoubleArray("DriveState/ModuleStates", m_moduleStatesArray);
        SignalLogger.writeDoubleArray("DriveState/ModuleTargets", m_moduleTargetsArray);
    }

    public void telemeterize(Elevator elevator, EndEffector endEffector) {
        elevatorSensorPosition.set(elevator.getPosition());
        elevatorSensorVelocity.set(elevator.getVelocity());
        elevatorDesiredPosition.set(elevator.getDesiredPosition());

        frontCanandcolorHit.set(endEffector.isFrontTriggered());
        backCanandcolorHit.set(endEffector.isBackTriggered());
    }
}