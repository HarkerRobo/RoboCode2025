package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AlignmentConstants {
    // Rotation constants
    // Blue letters come first
    private static final Rotation2d AB = new Rotation2d(Math.toRadians(0));
    private static final Rotation2d CD = new Rotation2d(Math.toRadians(60));
    private static final Rotation2d EF = new Rotation2d(Math.toRadians(120));
    private static final Rotation2d GH = new Rotation2d(Math.toRadians(180));
    private static final Rotation2d IJ = new Rotation2d(Math.toRadians(240));
    private static final Rotation2d KL = new Rotation2d(Math.toRadians(300));
    private static final Rotation2d RIGHTCORALSTATION = new Rotation2d(Math.toRadians(53));
    private static final Rotation2d LEFTCORALSTATION = new Rotation2d(Math.toRadians(-53));
    private static final Rotation2d BARGE = new Rotation2d(Math.toRadians(180));
    private static final Rotation2d PROCESSORANGLE = new Rotation2d(Math.toRadians(-90));

    private static final double LEFT_OFFSET = 0; //0.125;
    private static final double RIGHT_OFFSET = 0; //0.12;
    private static final double ROOT_3_2 = Math.sqrt(3)/2.0;


    public static final Pose2d REEF_A = new Pose2d(3.210, 3.924 + LEFT_OFFSET, AB);
   
    public static final Pose2d REEF_B = new Pose2d(3.210, 3.595 + RIGHT_OFFSET, AB);
   
    public static final Pose2d REEF_C = new Pose2d(3.938 - LEFT_OFFSET * ROOT_3_2, 2.867 + LEFT_OFFSET / 2.0, CD);
   
    public static final Pose2d REEF_D = new Pose2d(4.223 - RIGHT_OFFSET *ROOT_3_2, 2.702 + RIGHT_OFFSET / 2.0, CD);
   
    public static final Pose2d REEF_E = new Pose2d(5.218 - LEFT_OFFSET * ROOT_3_2, 2.969 - LEFT_OFFSET / 2.0, EF);
    
    public static final Pose2d REEF_F = new Pose2d(5.502 - RIGHT_OFFSET * ROOT_3_2, 3.133 - RIGHT_OFFSET / 2.0, EF);
    
    public static final Pose2d REEF_G = new Pose2d(5.769, 4.128 - LEFT_OFFSET, GH);
    
    public static final Pose2d REEF_H = new Pose2d(5.769, 4.457 - RIGHT_OFFSET, GH);
    
    public static final Pose2d REEF_I = new Pose2d(5.041 + LEFT_OFFSET * ROOT_3_2, 5.185 - LEFT_OFFSET / 2.0, IJ);
    
    public static final Pose2d REEF_J = new Pose2d(4.756 + RIGHT_OFFSET * ROOT_3_2, 5.350 - RIGHT_OFFSET / 2.0, IJ);
    
    public static final Pose2d REEF_K = new Pose2d(3.761 + LEFT_OFFSET * ROOT_3_2, 5.083 + LEFT_OFFSET / 2.0, KL);
    
    public static final Pose2d REEF_L = new Pose2d(3.476 + RIGHT_OFFSET * ROOT_3_2, 4.918 + RIGHT_OFFSET / 2.0, KL);
    
    public static final Pose2d CORAL1RIGHT = new Pose2d(1.536, 0.726, RIGHTCORALSTATION);
    public static final Pose2d CORAL1LEFT = new Pose2d(0.800, 1.258, RIGHTCORALSTATION);

    public static final Pose2d CORAL3RIGHT = new Pose2d(1.536, 7.277, LEFTCORALSTATION);
    public static final Pose2d CORAL3LEFT = new Pose2d(0.81, 6.802, LEFTCORALSTATION);

    public static final Pose2d BARGERIGHT = new Pose2d(8.510, 5.074, BARGE);
    public static final Pose2d BARGECENTER = new Pose2d(8.510, 6.168, BARGE);
    public static final Pose2d BARGELEFT = new Pose2d(8.510, 7.228, BARGE);

    public static final Pose2d PROCESSOR = new Pose2d(5.979, 0.541, PROCESSORANGLE);

    // Algae center positions
    public static final Pose2d ALGAE_AB = new Pose2d(3.210, 3.759, AB);

    public static final Pose2d ALGAE_CD = new Pose2d(4.081, 2.784, CD);

    public static final Pose2d ALGAE_EF = new Pose2d(5.360, 3.051, EF);

    public static final Pose2d ALGAE_GH = new Pose2d(5.769, 4.293, GH);

    public static final Pose2d ALGAE_IJ = new Pose2d(4.898, 5.267, IJ);
    
    public static final Pose2d ALGAE_KL = new Pose2d(3.619, 5.001, KL);

}