package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class AlignmentConstants {
    // Rotation constants
    // Blue letters come first
    private static final Rotation2d AB = new Rotation2d(Math.toRadians(0));
    private static final Rotation2d CD = new Rotation2d(Math.toRadians(60));
    private static final Rotation2d EF = new Rotation2d(Math.toRadians(120));
    private static final Rotation2d GH = new Rotation2d(Math.toRadians(180));
    private static final Rotation2d IJ = new Rotation2d(Math.toRadians(240));
    private static final Rotation2d KL = new Rotation2d(Math.toRadians(300));
    // private static final Rotation2d RIGHTCORALSTATION = new Rotation2d(Math.toRadians(53));
    // private static final Rotation2d LEFTCORALSTATION = new Rotation2d(Math.toRadians(-53));
    // private static final Rotation2d BARGE = new Rotation2d(Math.toRadians(180));
    // private static final Rotation2d PROCESSORANGLE = new Rotation2d(Math.toRadians(-90));


    private static final double LEFT_X_OFFSET = 0;
    private static final double RIGHT_X_OFFSET = 0;
    private static final double ALGAE_X_OFFSET = 0;
    private static final double Y_CORAL_OFFSET = 0.10;
    private static final double Y_ALGAE_OFFSET = -0.05;
    private static final Transform2d LEFT_OFFSET = new Transform2d(Y_CORAL_OFFSET, LEFT_X_OFFSET, new Rotation2d());
    private static final Transform2d RIGHT_OFFSET = new Transform2d(Y_CORAL_OFFSET, RIGHT_X_OFFSET, new Rotation2d());
    private static final Transform2d ALGAE_OFFSET = new Transform2d(Y_ALGAE_OFFSET, ALGAE_X_OFFSET, new Rotation2d());


    public static final Pose2d REEF_A = new Pose2d(3.210, 3.924, AB)
        .plus(LEFT_OFFSET);
   
    public static final Pose2d REEF_B = new Pose2d(3.210, 3.595, AB)
        .plus(RIGHT_OFFSET);
   
    public static final Pose2d REEF_C = new Pose2d(3.938, 2.867, CD)
        .plus(LEFT_OFFSET);
   
    public static final Pose2d REEF_D = new Pose2d(4.223, 2.702, CD)
        .plus(RIGHT_OFFSET);
   
    public static final Pose2d REEF_E = new Pose2d(5.218, 2.969, EF)
        .plus(LEFT_OFFSET);
    
    public static final Pose2d REEF_F = new Pose2d(5.502, 3.133, EF)
        .plus(RIGHT_OFFSET);
   
    public static final Pose2d REEF_G = new Pose2d(5.769, 4.128, GH)
    .plus(LEFT_OFFSET);
   
    public static final Pose2d REEF_H = new Pose2d(5.769, 4.457, GH)
    .plus(RIGHT_OFFSET);
    
    public static final Pose2d REEF_I = new Pose2d(5.041, 5.185, IJ)
        .plus(LEFT_OFFSET);
    
    public static final Pose2d REEF_J = new Pose2d(4.756, 5.350, IJ)
        .plus(RIGHT_OFFSET);
    
    public static final Pose2d REEF_K = new Pose2d(3.761, 5.083, KL)
        .plus(LEFT_OFFSET);
    
    public static final Pose2d REEF_L = new Pose2d(3.476, 4.918, KL)
        .plus(RIGHT_OFFSET);
    
    // public static final Pose2d CORAL1RIGHT = new Pose2d(1.536, 0.726, RIGHTCORALSTATION);
    // public static final Pose2d CORAL1LEFT = new Pose2d(0.800, 1.258, RIGHTCORALSTATION);

    // public static final Pose2d CORAL3RIGHT = new Pose2d(1.536, 7.277, LEFTCORALSTATION);
    // public static final Pose2d CORAL3LEFT = new Pose2d(0.81, 6.802, LEFTCORALSTATION);

    // public static final Pose2d BARGERIGHT = new Pose2d(8.510, 5.074, BARGE);
    // public static final Pose2d BARGECENTER = new Pose2d(8.510, 6.168, BARGE);
    // public static final Pose2d BARGELEFT = new Pose2d(8.510, 7.228, BARGE);

    // public static final Pose2d PROCESSOR = new Pose2d(5.979, 0.541, PROCESSORANGLE);

    // Algae center positions
    public static final Pose2d ALGAE_AB = new Pose2d(3.210, 3.759, AB)
        .plus(ALGAE_OFFSET);

    public static final Pose2d ALGAE_CD = new Pose2d(4.081, 2.784, CD)
        .plus(ALGAE_OFFSET);

    public static final Pose2d ALGAE_EF = new Pose2d(5.360, 3.051, EF)
        .plus(ALGAE_OFFSET);

    public static final Pose2d ALGAE_GH = new Pose2d(5.769, 4.293, GH)
        .plus(ALGAE_OFFSET);

    public static final Pose2d ALGAE_IJ = new Pose2d(4.898, 5.267, IJ)
        .plus(ALGAE_OFFSET);
    
    public static final Pose2d ALGAE_KL = new Pose2d(3.619, 5.001, KL)
        .plus(ALGAE_OFFSET);

}