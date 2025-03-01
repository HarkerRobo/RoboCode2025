package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Drivetrain;
import harkerrobolib.util.MathUtil;
import frc.robot.Limelight;
import frc.robot.LimelightHelpers;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

public class AutoAlign extends Command {
    private final Drivetrain drivetrain;
    private final SwerveRequest.RobotCentric drive;
    private int targetTagId = -1;
    private AlignmentTarget alignmentTarget;
    private AlignmentMode alignmentMode;
    
    // Alignment parameters
    private double targetDistance;
    private double lateralOffset = 0.0; // Offset to left or right of tag

    private double tx, ty;
    private double targetTx, targetTy;
    
    // PID Controllers
    private final PIDController xController = new PIDController(Constants.Drive.xAlignKP, Constants.Drive.xAlignKI, Constants.Drive.xAlignKD);
    private final PIDController yController = new PIDController(Constants.Drive.yAlignKP, Constants.Drive.yAlignKI, Constants.Drive.yAlignKD);
    private final PIDController rotationController = new PIDController(Constants.Drive.rotAlignKP, Constants.Drive.rotAlignKI, Constants.Drive.rotAlignKD);
    
    public enum AlignmentTarget {
        REEF,
        BARGE,
        ANY
    }
    
    // Alignment modes for coral
    public enum AlignmentMode {
        CENTER, // Align directly with the AprilTag
        LEFT_POLE, // Align with the left pole next to the tag
        RIGHT_POLE // Align with the right pole next to the tag
    }
    
    /**
     * Creates an auto-alignment command with a specific target
     * @param drivetrain The swerve drivetrain to control
     * @param target The field element to align with
     * @param mode The alignment mode (for CORAL targets)
     */
    public AutoAlign(Drivetrain drivetrain, AlignmentTarget target, AlignmentMode mode) {
        this.drivetrain = drivetrain;
        this.alignmentTarget = target;
        this.alignmentMode = mode;
        
        // Set target distance based on the field element
        switch (target) {
            case REEF:
                this.targetDistance = Constants.Drive.REEF_TARGET_DISTANCE;
                
                // Set lateral offset based on alignment mode
                if (mode == AlignmentMode.LEFT_POLE) {
                    this.lateralOffset = Constants.Drive.CORAL_POLE_OFFSET;
                } else if (mode == AlignmentMode.RIGHT_POLE) {
                    this.lateralOffset = -Constants.Drive.CORAL_POLE_OFFSET;
                }
                break;
            case BARGE:
                this.targetDistance = Constants.Drive.BARGE_TARGET_DISTANCE;
                break;
            case ANY:
            default:
                this.targetDistance = Constants.Drive.DEFAULT_TARGET_DISTANCE;
                break;
        }
        
        this.drive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);
        
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drivetrain);
    }
    
    /**
     * Creates an auto-alignment command to the coral with a specific mode
     * @param drivetrain The swerve drivetrain to control
     * @param mode The alignment mode (CENTER, LEFT_POLE, RIGHT_POLE)
     */
    public AutoAlign(Drivetrain drivetrain, AlignmentMode mode) {
        this(drivetrain, AlignmentTarget.REEF, mode);
    }
    
    /**
     * Creates an auto-alignment command using the default Limelight to any visible tag
     * @param drivetrain The swerve drivetrain to control
     */
    public AutoAlign(Drivetrain drivetrain) {
        this(drivetrain, AlignmentTarget.ANY, AlignmentMode.CENTER);
    }

    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        rotationController.reset();
        targetTagId = -1;
        
        // Log the start of alignment
        SmartDashboard.putBoolean("AutoAlign/Active", true);
        SmartDashboard.putString("AutoAlign/Target", alignmentTarget.toString());
        SmartDashboard.putString("AutoAlign/Mode", alignmentMode.toString());
    }

    @Override
    public void execute() {
    

        // Get current tag ID
        int currentTagId = (int) Limelight.getApriltagId();
        if (currentTagId <= 0) {
            // Invalid tag ID
            drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            return;
        }

        // Initialize targetTagId if this is the first detection
        if (targetTagId == -1) {
            targetTagId = currentTagId;
            
            // Update the target if needed based on the detected tag
            if (alignmentTarget == AlignmentTarget.ANY) {
                // Automatically determine target type based on tag ID
                updateAlignmentTargetFromTagId(currentTagId);
            }
        }
        
        // Check if target tag is still in view
        if (targetTagId != currentTagId) {
            // Target tag changed, reset
            targetTagId = currentTagId;
            if (alignmentTarget == AlignmentTarget.ANY) {
                updateAlignmentTargetFromTagId(currentTagId);
            }
        }
        
        // Get target offsets
        tx = Limelight.getTx();
        ty = Limelight.getTy();
        
        // Calculate speeds
        double xSpeed = 0;
        double ySpeed = 0;
        double rotSpeed = 0;

        // Rotation (keep tag centered horizontally, with offset for pole alignment)
        targetTx = 0.0;
        
        // Apply offset for pole alignment
        if (alignmentTarget == AlignmentTarget.REEF) {
            if (alignmentMode == AlignmentMode.LEFT_POLE) {
                // Offset to left pole (positive tx is right, so negative offset to look left)
                targetTx = -Constants.Drive.CORAL_POLE_TX_OFFSET;
            } else if (alignmentMode == AlignmentMode.RIGHT_POLE) {
                // Offset to right pole
                targetTx = Constants.Drive.CORAL_POLE_TX_OFFSET;
            }
        }
        
        if (Math.abs(tx-targetTx) > Constants.Drive.ALIGN_ERROR) {
            ySpeed = yController.calculate(tx, targetTx);
        }
        // Forward/back (maintain target distance using ty as a proxy)
        // double targetTy = 0.0;
        // switch (alignmentTarget) {
        //     case REEF:
        //         targetTy = Constants.Drive.REEF_TARGET_TY;
        //         break;
        //     default:
        //         targetTy = -14.0;  // Default value
        // }
        
        // if (Math.abs(ty - targetTy) > 1.0) {
        //     xSpeed = xController.calculate(ty, targetTy);
        // }
        
        // // Lateral alignment for pole targeting (simple strafing based on alignment mode)
        // if (alignmentTarget == AlignmentTarget.REEF && Math.abs(tx - targetTx) < 5.0) {
        //     // Only start strafing when somewhat aligned rotationally
        //     if (alignmentMode == AlignmentMode.LEFT_POLE) {
        //         ySpeed = Constants.Drive.MAX_TRANSLATION_SPEED * 0.3; // Strafe left
        //     } else if (alignmentMode == AlignmentMode.RIGHT_POLE) {
        //         ySpeed = -Constants.Drive.MAX_TRANSLATION_SPEED * 0.3; // Strafe right
        //     }
        // }
        
        // Apply control with limits
        drivetrain.setControl(drive
            .withVelocityY(clamp(ySpeed, -Constants.Drive.MAX_TRANSLATION_SPEED, Constants.Drive.MAX_TRANSLATION_SPEED)));
            // .withVelocityX(clamp(ySpeed, -Constants.Drive.MAX_TRANSLATION_SPEED, Constants.Drive.MAX_TRANSLATION_SPEED))
            // .withRotationalRate(clamp(rotSpeed, -Constants.Drive.MAX_ROTATION_SPEED, Constants.Drive.MAX_ROTATION_SPEED)));
            
        // Log debug values
        SmartDashboard.putNumber("AutoAlign/TX", tx);
        SmartDashboard.putNumber("AutoAlign/TargetTX", targetTx);
        // SmartDashboard.putNumber("AutoAlign/TY", ty);
        // SmartDashboard.putNumber("AutoAlign/TargetTY", targetTy);
        SmartDashboard.putNumber("AutoAlign/XSpeed", xSpeed);
        // SmartDashboard.putNumber("AutoAlign/YSpeed", ySpeed);
        // SmartDashboard.putNumber("AutoAlign/RotSpeed", rotSpeed);
    }
    
    /**
     * Update the alignment target based on the detected tag ID
     */
    private void updateAlignmentTargetFromTagId(int tagId) {
        // In 2025, tags 1-16 are on the coral structures TODO
        if (tagId >= 1 && tagId <= 16) {
            alignmentTarget = AlignmentTarget.REEF;
            targetDistance = Constants.Drive.REEF_TARGET_DISTANCE;
        }
        
        SmartDashboard.putString("AutoAlign/Target", alignmentTarget.toString());
    }
    
    
    /**
     * Calculate the desired heading based on tag ID and target pose
     */
    private double calculateDesiredHeading(int tagId, Pose3d targetPose3d) {
        // Base heading (pointing directly at the tag)
        double baseHeading = Math.atan2(targetPose3d.getY(), targetPose3d.getX());
        
        // For coral structure, we want to approach perpendicular to the face
        if (alignmentTarget == AlignmentTarget.REEF) {
            // The coral faces are aligned with the tags, so we approach directly
            return baseHeading;
        }
        
        // Default alignment - face the tag directly
        return baseHeading;
    }

    public boolean isFinished() {
        return MathUtil.compareSetpoint(tx, targetTx, Constants.Drive.ALIGN_ERROR);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        SmartDashboard.putBoolean("AutoAlign/Active", false);
    }
    
    /**
     * Calculate the rotation error, normalized to the range [-π, π]
     */
    private double getRotationError(double current, double target) {
        double error = target - current;
        // Normalize to [-π, π]
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;
        return error;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}