package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.pathplanner.lib.path.PathConstraints;

public class Constants {

    public static final String CAN_CHAIN = "rio";
    public static final double MAX_VOLTAGE = 12;
    public static final double ROBOT_LOOP = 0.02;

    public static class Vision {
        public static final String kCamera1Name = "limelight";
        
        public static final Transform3d kRobotToCam1 = new Transform3d(
                new Translation3d(Units.inchesToMeters(5.472), Units.inchesToMeters(-10.5), Units.inchesToMeters(7.482)),
                new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(26.1), 0)); // new mount

        public static final String kCamera2Name = "limelight-upper";
        public static final Transform3d kRobotToCam2 = new Transform3d(
            new Translation3d(Units.inchesToMeters(2.507), Units.inchesToMeters(-2.725), Units.inchesToMeters(36.425)), // TODO
            new Rotation3d(0, Units.degreesToRadians(20.55), 180)); // new mount

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.kDefaultField);

        // The standard deviations of our vision estimated poses, which affect
        // correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kTagStdDevs = VecBuilder.fill(0.3, 0.1, 0.5);
        public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.05, 0.1);
        // public static final Matrix<N3, N1> kTagStdDevs = VecBuilder.fill(1.0, 0.5, 0.5);
        // public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1,1.0, 0.1);

        public static final PathConstraints constraints = new PathConstraints(
        2.0, 2.0,
            Units.degreesToRadians(360), Units.degreesToRadians(540));
    }

    public static class Sim {
        public static final double kSimLoopPeriod = 0.02; // 5 ms
    }

    public static class Drive {
        /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
        public static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
        /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
        public static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

        public static final double MAX_TRANSLATION_SPEED = 1.0; // m/s
        public static final double MAX_ROTATION_SPEED = 1.0; // rad/s

        public static final double xAlignKP = 1.0; // Forward/backward control gain
        public static final double xAlignKI = 0; // Integral gain
        public static final double xAlignKD = 0; // Derivative gain for smoothing

        public static final double yAlignKP = 0.3; // Side-to-side control gain
        public static final double yAlignKI = 0; // Integral gain
        public static final double yAlignKD = 0; // Derivative gain for smoothing

        public static final double rotAlignKP = 0.1; // Rotation control gain (radians)
        public static final double rotAlignKI = 0; // Integral gain
        public static final double rotAlignKD = 0; // Derivative gain for smoothing

        public static final double REEF_TARGET_DISTANCE = 0; // meters from reef face
        public static final double BARGE_TARGET_DISTANCE = 2.0; // meters from speaker
        public static final double DEFAULT_TARGET_DISTANCE = 1.0; // default approach distance

        // Coral structure alignment specifics
        public static final double CORAL_POLE_OFFSET = Meters.convertFrom(6.074, Inches); // meters to offset from center to poles 6.074 in
        public static final double CORAL_POLE_TX_OFFSET = 12.0; // approximate tx value offset for poles

        // Target ty values for different game elements (for tx/ty fallback mode)
        public static final double REEF_TARGET_TY = -14.0; // ty value for reef approach

        public static final double ALIGN_ERROR = 0.2;
    }

    /**
     * =========================================================================
     * ALL THE CONSTANTS IN THE SWERVE SUB-CLASS NEED TO BE ADJUSTED ACCORDINGLY
     * GET ALL CONSTANTS FROM CTRE'S SWERVE GENERATOR
     * =========================================================================
     */
    public static class Swerve { // ALL THESE IN THIS SUB-CLASS NEED TO BE RE-TUNED (get from running
                                 // swervegenerator again)
        public static final Slot0Configs steerGains = new Slot0Configs()
                .withKP(30).withKI(0).withKD(0.5)
                .withKS(0.1).withKV(0).withKA(0.047118)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        public static final Slot0Configs driveGains = new Slot0Configs()
                .withKP(1).withKI(0).withKD(0)
                .withKS(0.096083).withKV(0.11371).withKA(0.020463);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        public static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        public static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The type of motor used for the drive motor
        public static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
        // The type of motor used for the drive motor
        public static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

        // The remote sensor feedback type to use for the steer motors;
        // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
        public static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        public static final Current kSlipCurrent = Amps.of(120.0);

        // Initial configs for the drive and steer motors and the azimuth encoder; these
        // cannot be null.
        // Some configs will be overwritten; check the `with*InitialConfigs()` API
        // documentation.
        public static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
        public static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                // Swerve azimuth does not require much torque output,
                                // so we can set a relatively low
                                // stator current limit to help avoid brownouts without
                                // impacting performance.
                                .withStatorCurrentLimit(Amps.of(60))
                                .withStatorCurrentLimitEnable(true));
        public static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
        // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
        public static final Pigeon2Configuration pigeonConfigs = new Pigeon2Configuration().withMountPose(new MountPoseConfigs().withMountPoseYaw(180));

        // CAN bus that the devices are located on;
        // All swerve devices must share the same CAN bus
        public static final CANBus kCANBus = new CANBus("rio", "/home/lvuser/logs");

        // Theoretical free speed (m/s) at 12 V applied output;
        // This needs to be tuned to your individual robot
        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(8.94);

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        public static final double kCoupleRatio = 0;

        public static final double kDriveGearRatio = 7.14;
        public static final double kSteerGearRatio = 21.429;
        public static final Distance kWheelRadius = Inches.of(4);

        public static final boolean kInvertLeftSide = false;
        public static final boolean kInvertRightSide = true;

        public static final int kPigeonId = 1;

        // These are only used for simulation
        public static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
        public static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
        // Simulated voltage necessary to overcome friction
        public static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
        public static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

        // Front Left
        public static final int kFrontLeftDriveMotorId = 3;
        public static final int kFrontLeftSteerMotorId = 1;
        public static final int kFrontLeftEncoderId = 2;
        public static final Angle kFrontLeftEncoderOffset = Rotations.of(0.028564453125);
        public static final boolean kFrontLeftSteerMotorInverted = true;
        public static final boolean kFrontLeftEncoderInverted = false;

        public static final Distance kFrontLeftXPos = Inches.of(14);
        public static final Distance kFrontLeftYPos = Inches.of(15);

        // Front Right
        public static final int kFrontRightDriveMotorId = 6;
        public static final int kFrontRightSteerMotorId = 4;
        public static final int kFrontRightEncoderId = 5;
        public static final Angle kFrontRightEncoderOffset = Rotations.of(0.06689453125);
        public static final boolean kFrontRightSteerMotorInverted = true;
        public static final boolean kFrontRightEncoderInverted = false;

        public static final Distance kFrontRightXPos = Inches.of(14);
        public static final Distance kFrontRightYPos = Inches.of(-15);

        // Back Left
        public static final int kBackLeftDriveMotorId = 13;
        public static final int kBackLeftSteerMotorId = 11;
        public static final int kBackLeftEncoderId = 12;
        public static final Angle kBackLeftEncoderOffset = Rotations.of(0.379150390625);
        public static final boolean kBackLeftSteerMotorInverted = true;
        public static final boolean kBackLeftEncoderInverted = false;

        public static final Distance kBackLeftXPos = Inches.of(-14);
        public static final Distance kBackLeftYPos = Inches.of(15);

        // Back Right
        public static final int kBackRightDriveMotorId = 9;
        public static final int kBackRightSteerMotorId = 8;
        public static final int kBackRightEncoderId = 10;
        public static final Angle kBackRightEncoderOffset = Rotations.of(0.38525390625);
        public static final boolean kBackRightSteerMotorInverted = true;
        public static final boolean kBackRightEncoderInverted = false;

        public static final Distance kBackRightXPos = Inches.of(-14);
        public static final Distance kBackRightYPos = Inches.of(-15);
    }

    public static final class Elevator {
        public static final double MAX_ERROR = 0.05; // TODO rotations

        public static final double kP = 10;
        public static final double kI = 0;
        public static final double kD = 0.2;

        public static final double kG = 0.42279;
        public static final double kV = 0.59985;
        public static final double kS = 0.19926;

        public static final InvertedValue MASTER_INVERTED = InvertedValue.Clockwise_Positive;
        public static final InvertedValue FOLLOWER_INVERTED = InvertedValue.Clockwise_Positive;

        public static final int MASTER_ID = 14;
        public static final int FOLLOWER_ID = 15;
        public static final int LIMIT_SWITCH_ID = 0;

        public static final double STATOR_CURRENT_LIMIT = 90;
        public static final double SUPPLY_CURRENT_LIMIT = 90;
        public static final double FORWARD_SOFT_LIMIT = 4.7; // otations
        public static final double REVERSE_SOFT_LIMIT = -0.1; // rotations

        public static final double ZERO_SPEED = -0.1;

        public static final double ELEVATOR_STALLING_CURRENT = 80;

        public static final double ELEVATOR_GEAR_RATIO = 6.22;

        public static final double[] CORAL_HEIGHTS = { 0, 1.45, 2.85, 4.69 }; // rotations
        public static final double[] ALGAE_HEIGHTS = { 1.993, 3.149 }; // low, high

        public static final double MM_CRUISE_VELOCITY = 40; // rps
        public static final double MM_ACCELERATION = 60; // rps/s
        public static final double MM_JERK = 240; // rps/s/s
    }

    public static final class EndEffector // positive output = out, negative = in
    {
        public static final int ID = 16;
        public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;
        public static final int BACK_CANANDCOLOR_ID = 1;
        public static final int FRONT_CANANDCOLOR_ID = 2;

        public static final double STATOR_CURRENT_LIMIT = 80;

        public static final double PROXIMITY_LIMIT_FRONT = 0.23;
        public static final double PROXIMITY_LIMIT_BACK = 0.15;

        public static final double INTAKE_ALGAE_SPEED = 0.4;
        public static final double INTAKE_CORAL_SPEED = -0.3;
        public static final double INTAKE_CORAL_SLOW_SPEED = -0.15;
        public static final double OUTTAKE_SPEED = -0.3;
        public static final double EJECT_SPEED = 0.1;

        public static final double ALGAE_HOLD_SPEED = 0.15;
    }

    public static final class Climb {
        public static final int ID = 17;

        public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive; // TODO

        public static final double CLIMB_GEAR_RATIO = 23.7;

        public static final double kP = 1; // TODO

        public static final double STATOR_CURRENT_LIMIT = 90; // TODO

        public static final double SUPPLY_CURRENT_LIMIT = 90; // TODO

        public static final double CLIMB_POSITION = 1.0;  // rotations // TODO

        public static final double MAX_ERROR = 0.1;
    }
}
