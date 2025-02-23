package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class RobotMap 
{

    // Global Robot Constants
    public static final double MAX_VOLTAGE = 12;
    public static final double ROBOT_LOOP = 0.02;
    public static final String CAN_CHAIN = "rio";

    public static final class OI 
    {
        public static final double JOYSTICK_DEADBAND = 0.15;
        public static final double TRIGGER_DEADBAND = 0.1;
      
        public static final int DRIVER_ID = 0;
        public static final int OPERATOR_ID = 1;
    }

    public static final class PID 
    {
        public static final int PID_PRIMARY = 0;
        public static final int PID_AUXILIARY = 1;
      
        public static final int SLOT_INDEX = 0;
    }

    public static final class Field {
        // field dimensions in meters
        public static final double FIELD_LENGTH = 17.55;
        public static final double FIELD_WIDTH = 8.05;
        public static final Field2d FIELD = new Field2d();
    }

    public static final class SwerveModule 
    {
        // id of translation motors
        // FL, FR, BL, BR
        public static final int[] TRANSLATION_IDS = {3, 6, 13, 9};

        // translation motors inverted
        public static final InvertedValue[] TRANSLATION_INVERTS = new InvertedValue[]{InvertedValue.CounterClockwise_Positive, InvertedValue.CounterClockwise_Positive, InvertedValue.CounterClockwise_Positive, InvertedValue.CounterClockwise_Positive};

        // ids for rotation motors
        public static final int[] ROTATION_IDS = {1, 4, 11, 8};

        // rotation motors inverted
        public static final InvertedValue[] ROTATION_INVERTS = {InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive};

        // cancoder ids
        public static final int[] CAN_CODER_ID = {2, 5, 12, 10};

        // offsets of cancoders of each swerve module (in rotations)
        public static final double[] CAN_CODER_OFFSETS = new double[]{0.474609, -0.076904, -0.381104+0.5, -0.385742};
        // current limit constants for translation motors
        public static final double TRANS_LOWER_CURRENT_LIMIT = 40;
        public static final double TRANS_CURRENT_LIMIT = 60;
        public static final double TRANS_LOWER_LIMIT_TIME= 0.1;

        // current limit constants for rotation motors
        public static final double ROT_LOWER_CURRENT_LIMIT = 25;
        public static final double ROT_CURRENT_LIMIT = 40;
        public static final double ROT_LOWER_LIMIT_TIME = 0.1;

        // gear ratios
        public static final double TRANSLATION_GEAR_RATIO = 6.696;
        public static final double ROTATION_GEAR_RATIO = 150.0 / 7.0; 
        // diameter of the wheel
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0); // meters

        // conversions from rotations -- NO GEAR RATIOS!!!
        public static final double TRANS_ROT_TO_METERS = WHEEL_DIAMETER * Math.PI; // rotations to meters
        public static final double ROT_ROT_TO_ANGLE = 360.0; // rotations to degrees

        // rotation kP
        public static final double ROTATION_kP = 50; //50; // TODO
        public static final double ROTATION_kI = 0;
        public static final double ROTATION_kD = 0;

        // Translation FF Values
        public static final double TRANSLATION_kS = 0.13561; // TODO
        public static final double TRANSLATION_kV = 1.9051; // TODO
        public static final double TRANSLATION_kA = 1.5737; // TODO

        // pid
        public static final double TRANSLATION_kP = 1.7; //1.7; // TODO
        public static final double TRANSLATION_kI = 0.5; //0.5; // TODO
        public static final double TRANSLATION_kD = 0.0; //0.00;  // TODO
    }

    public static final class Drivetrain 
    {
        // Pigeon ID
        public static final int PIGEON_ID = 1;

        public static final double PIGEON_kP = 0.001; // TODO

        public static final double MIN_OUTPUT = 0.1;
        
        // Robot Dimensions
        public static final double ROBOT_LENGTH = Units.inchesToMeters(30);
        public static final double ROBOT_WIDTH = Units.inchesToMeters(28);

        public static final double MAX_DRIVING_SPEED = 5.0; // m/s //TODO
        public static final double MAX_ACCELERATION = 10.0;
        public static final double MAX_SLOW_DRIVING_SPEED = 1.0;
        public static final double MAX_SLOW_ACCELERATION = 1.5;
        public static final double MAX_ANGLE_VELOCITY = Math.PI;
        public static final double MAX_SLOW_ANGLE_VELOCITY = Math.PI * 0.6;
        public static final double MAX_ANGLE_ACCELERATION = MAX_ANGLE_VELOCITY / 2.0;

    }

	public static final class Elevator 
    {
		public static final double MAX_ERROR = 0.05; // TODO rotations
												
		public static final double kP = 3.596;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

		public static final double kG = 0.41279; // TODO
        public static final double kV = 0.69985;
        public static final double kS = 0.19926;

        public static final InvertedValue MASTER_INVERTED = InvertedValue.Clockwise_Positive; // TODO
        public static final InvertedValue FOLLOWER_INVERTED = InvertedValue.Clockwise_Positive; // TODO

        public static final int MASTER_ID = 14;
        public static final int FOLLOWER_ID = 15;
        public static final int LIMIT_SWITCH_ID = 0; // TODO

        public static final double STATOR_CURRENT_LIMIT = 90; // TODO 
        public static final double SUPPLY_CURRENT_LIMIT = 90;
        public static final double FORWARD_SOFT_LIMIT = 4.8; // TODO rotations
        public static final double REVERSE_SOFT_LIMIT = -0.1; // TODO rotations

        public static final double ZERO_SPEED = -0.1;

        public static final double ELEVATOR_STALLING_CURRENT = 80;

        public static final double ELEVATOR_GEAR_RATIO = 6.22; // TODO

        public static final double[] LEVEL_HEIGHTS = {0, 1.45, 2.85, 4.75}; // TODO rotations
        public static final double[] ALGAE_HEIGHTS = {1.993, 3.149}; // TODO low, high
    }
    
    public static final class EndEffector // positive output = out, negative = in
    {
        public static final int ID = 16; // TODO
        public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive; // TODO
        public static final int BACK_CANANDCOLOR_ID = 1; // TODO
        public static final int FRONT_CANANDCOLOR_ID = 2; // TODO

        public static final double kP = 2; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 0; // TODO

        public static final double STATOR_CURRENT_LIMIT = 80; // TODO A

        public static final double PROXIMITY_LIMIT_FRONT = 0.23; // TODO
        public static final double PROXIMITY_LIMIT_BACK = 0.15;

        public static final double INTAKE_ALGAE_SPEED = 0.4;
        public static final double INTAKE_CORAL_SPEED = -0.3;
        public static final double OUTTAKE_SPEED = -0.3;
        public static final double EJECT_SPEED = 0.1;

        public static final double ALGAE_HOLD_SPEED = 0.15;
    }

    public static final class Climb {
        public static final int ID = 999; // TODO

        public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive; // TODO

        public static final double CLIMB_GEAR_RATIO = 23.7;
        
        public static final double kP = 1; // TODO

        public static final double STATOR_CURRENT_LIMIT = 90; // TODO

        public static final double SUPPLY_CURRENT_LIMIT = 90; // TODO

        public static final double RESET_POSITION = 1; // TODO

        public static final double CLIMB_VELOCITY = -0.5; // TODO
    }
}
