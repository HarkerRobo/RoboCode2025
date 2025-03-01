package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import harkerrobolib.util.MathUtil;

public final class Limelight {
    private static final String LIMELIGHT_TABLE_KEY = "limelight-upper";
    private static final NetworkTable TABLE = NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_KEY);

    public static boolean hasTargets() {
        return MathUtil.compareDouble(TABLE.getEntry("tv").getDouble(0.0), 1.0);
    }
    
    /* entries[0] = forward;
     * entries[1] = side;
     * entries[2] = up;
     * entries[3] = roll;
     * entries[4] = pitch;
     * entries[5] = yaw; */
    public static void setCameraPose(Transform3d pose) {
        TABLE.getEntry("camerapose_robotspace_set").setDoubleArray(new double[]{pose.getX(), pose.getY(), pose.getZ(), pose.getRotation().getX(), pose.getRotation().getY(), pose.getRotation().getZ()});

    }

    public static void setPipeline(double idx) {
        TABLE.getEntry("pipeline").setDouble(idx);
    }

    public static double getApriltagId() {
        return TABLE.getEntry("tid").getDouble(0.0);
    }

    public static double getTx() {
        return TABLE.getEntry("tx").getDouble(0.0);
    }

    public static double getTy() {
        return TABLE.getEntry("ty").getDouble(0.0);
    }

    public static Pose3d getTargetPose3d_RobotSpace() {
        double[] poseArray = TABLE.getEntry("targetpose_robotspace").getDoubleArray(new double[0]);
        return toPose3D(poseArray);
    }

    public static Pose3d getTargetPose3d_CameraSpace() {
        double[] poseArray = TABLE.getEntry("targetpose_cameraspace").getDoubleArray(new double[0]);
        return toPose3D(poseArray);
    }

    /**
     * Takes a 6-length array of pose data and converts it to a Pose3d object.
     * Array format: [x, y, z, roll, pitch, yaw] where angles are in degrees.
     * @param inData Array containing pose data [x, y, z, roll, pitch, yaw]
     * @return Pose3d object representing the pose, or empty Pose3d if invalid data
     */
    public static Pose3d toPose3D(double[] inData){
        if(inData.length < 6)
        {
            //System.err.println("Bad LL 3D Pose Data!");
            return new Pose3d();
        }
        return new Pose3d(
            new Translation3d(inData[0], inData[1], inData[2]),
            new Rotation3d(Units.degreesToRadians(inData[3]), Units.degreesToRadians(inData[4]),
                    Units.degreesToRadians(inData[5])));
    }
}