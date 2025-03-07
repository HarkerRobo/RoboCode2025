// code from team 5675

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AlignmentConstants;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.AlignDirection;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.swerve.Drivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveToPoseCommand extends Command {
    private final Drivetrain drivetrain;
    private Pose2d targetPose;
    private Command pathCommand;
    private boolean isBarge;
    private double aprilTagId;

    public DriveToPoseCommand(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
}

    @Override
    public void initialize() {
        AlignDirection direction = RobotContainer.getInstance().getAlignDirection();
        isBarge = direction == AlignDirection.MidBarge || direction == AlignDirection.LeftBarge || direction == AlignDirection.RightBarge;
        System.out.println("Starting DriveToPoseCommand...");
        updateTargetPose();
        startPath();
    }

    @Override
    public void execute() {
        if (pathCommand != null) {
            pathCommand.execute(); 
        }

        SmartDashboard.putString("Drive/direction", RobotContainer.getInstance().getAlignDirection().toString());
        SmartDashboard.putBoolean("Drive/isBarge", isBarge);
        SmartDashboard.putNumber("Drive/aprilTagId", aprilTagId);
    }

    @Override
    public boolean isFinished() {
        return pathCommand == null || pathCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (pathCommand != null) {
            pathCommand.cancel();
            System.out.println("DriveToPoseCommand finished.");
            }
    }

    /** Updates the target pose dynamically based on AprilTag ID */
    private void updateTargetPose() {

        if (targetPose != null && drivetrain.getState().Pose.equals(targetPose)) {
            System.out.println("Already at target pose. No path needed.");
            pathCommand = null; 
            return;
        }
        aprilTagId = LimelightHelpers.getFiducialID(Constants.Vision.kCamera1Name);
        
        if (aprilTagId == -1) {
            System.out.println("No valid AprilTag detected.");
            return;
        } else {
            targetPose = getTargetPose((int) aprilTagId);
        }

        // Flip pose if we're on the red alliance
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) targetPose = FlippingUtil.flipFieldPose(targetPose);
        
    }

    private void startPath() {
        if (targetPose != null) {
            Pose2d currentPose = drivetrain.getState().Pose;
            
            // Check if the robot is already at the target position within a small tolerance
            double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
            double angleDifference = Math.abs(currentPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees());
    
            if (distance < 0.05 && angleDifference < 2) { // 5 cm and 2 degrees tolerance
                System.out.println("Already at target pose. No path needed.");
                pathCommand = null;
                return;
            }
            
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(currentPose, targetPose);
    
            if (waypoints.isEmpty()) {
                System.out.println("No waypoints generated. Skipping path.");
                pathCommand = null;
                return;
            }

            if (isBarge) {
                PathPlannerPath bargePath = getBargePath();
                
                    if (bargePath == null) {
                        System.out.println("Error: Barge path is null. Skipping execution.");
                        pathCommand = null;
                        return; 
                    }
                //if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) bargePath = bargePath.flipPath();
                pathCommand = AutoBuilder.pathfindThenFollowPath(bargePath, Constants.Vision.constraints);

            } else {

                PathPlannerPath generatedPath = new PathPlannerPath(waypoints, 
                Constants.Vision.constraints, null, 
                new GoalEndState(0, targetPose.getRotation()));
                generatedPath.preventFlipping = true;
                pathCommand = AutoBuilder.followPath(generatedPath);
            }

            if (pathCommand == null) {
                System.out.println("PathPlanner failed to generate a command. Skipping execution.");
                return;
            }
    
            try {
                pathCommand.initialize();
            } catch (Exception e) {
                e.printStackTrace();
                return;
            }
        }
    }

    /** Returns the correct target pose based on AprilTag ID and direction */
    private Pose2d getTargetPose(int aprilTagId) {

        return switch (RobotContainer.getInstance().getAlignDirection()) {
            case Left -> switch (aprilTagId) {
                case 18, 7 -> AlignmentConstants.REEF_A;
                case 19, 6 -> AlignmentConstants.REEF_K;
                case 20, 11 -> AlignmentConstants.REEF_I;
                case 21, 10 -> AlignmentConstants.REEF_G;
                case 22, 9 -> AlignmentConstants.REEF_E;
                case 17, 8 -> AlignmentConstants.REEF_C;
                case 12, 2 -> AlignmentConstants.CORAL1LEFT;
                case 13, 1 -> AlignmentConstants.CORAL3LEFT;
                case 3, 16 -> AlignmentConstants.PROCESSOR;
                default -> {
                    System.out.println("Unknown AprilTag ID for left: " + aprilTagId);
                    yield drivetrain.getState().Pose;
                }
            };
            case Right -> switch (aprilTagId) {
                case 18, 7 -> AlignmentConstants.REEF_B;
                case 19, 6 -> AlignmentConstants.REEF_L;
                case 20, 11 -> AlignmentConstants.REEF_J;
                case 21, 10 -> AlignmentConstants.REEF_H;
                case 22, 9 -> AlignmentConstants.REEF_F;
                case 17, 8 -> AlignmentConstants.REEF_D;
                case 12, 2 -> AlignmentConstants.CORAL1RIGHT;
                case 13, 1 -> AlignmentConstants.CORAL3RIGHT;
                case 3, 16 -> AlignmentConstants.PROCESSOR;
                default -> {
                    System.out.println("Unknown AprilTag ID for right: " + aprilTagId);
                    yield drivetrain.getState().Pose;
                }
            };
            case Algae -> switch (aprilTagId) {
                case 18, 7 -> AlignmentConstants.ALGAE_AB;
                case 19, 6 -> AlignmentConstants.ALGAE_KL;
                case 20, 11 -> AlignmentConstants.ALGAE_IJ;
                case 21, 10 -> AlignmentConstants.ALGAE_GH;
                case 22, 9 -> AlignmentConstants.ALGAE_EF;
                case 17, 8 -> AlignmentConstants.ALGAE_CD;
                default -> {
                    System.out.println("Unknown AprilTag ID for algae: " + aprilTagId);
                    yield drivetrain.getState().Pose;
                }
            };
            default -> drivetrain.getState().Pose;
        };
    }
   private PathPlannerPath getBargePath(){
        AlignDirection direction = RobotContainer.getInstance().getAlignDirection();
        try {
            return switch (direction) {
                case MidBarge -> PathPlannerPath.fromPathFile("MidBarge");
                case LeftBarge -> PathPlannerPath.fromPathFile("LeftBarge");
                case RightBarge -> PathPlannerPath.fromPathFile("RightBarge");
                default -> throw new IllegalStateException("Invalid barge direction: " + direction);
            };
        } catch (Exception e) {
            System.err.println("Error loading PathPlanner path for direction: " + direction);
            e.printStackTrace();
            return null;
        }
    }
}