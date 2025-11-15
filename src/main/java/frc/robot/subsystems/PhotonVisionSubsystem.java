package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EnabledSubsystems;
import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.RobotContainer;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public class PhotonVisionSubsystem extends SubsystemBase {
    
    // ==================== CONSTANTS ====================
    
    /** Camera configuration */
public static final class CameraConstants {
    // Camera network info
    public static final String CAMERA_NAME = "Front_OBJ_Arducam_OV9782";  // ✅ UPDATED
    public static final String CAMERA_IP = "10.51.42.11";                 // ✅ UPDATED
    
    // Camera mounting position relative to robot center
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(9.75);   // ✅ UPDATED: 9.75" off ground
    public static final double CAMERA_PITCH_DEGREES = 0;  // Angle above horizontal (tune if camera is tilted)
    public static final double CAMERA_YAW_DEGREES = 0.0;    // Angle left/right of forward
    
    // Camera offset from robot center (in robot coordinate system)
    // Robot coordinate system: +X = forward, +Y = left, +Z = up
    public static final double CAMERA_FORWARD_OFFSET_METERS = Units.inchesToMeters(12.0);   // ✅ UPDATED: 12" forward
    public static final double CAMERA_SIDE_OFFSET_METERS = Units.inchesToMeters(5.5);       // ✅ UPDATED: 5.5" LEFT of center
    public static final double CAMERA_VERTICAL_OFFSET_METERS = Units.inchesToMeters(9.75);  // ✅ ADDED: Same as height
}
    
public static final class DetectionConstants {
    // Pipeline indices (set these in PhotonVision web interface)
    public static final int CORAL_PIPELINE = 0;      // Object Detection pipeline
    public static final int ALGAE_PIPELINE = 1;      // Object Detection or Colored Shape
    public static final int APRIL_TAG_PIPELINE = 2;  // AprilTag pipeline
    
    // Detection filtering
    public static final double MIN_CONFIDENCE = 0.5;  // ✅ Minimum detection confidence (0-1)
    public static final double MIN_TARGET_AREA = 0.05;  // ✅ Reduced for object detection (less strict)
    public static final double MAX_DETECTION_RANGE_METERS = 5.0;  // Max detection distance
    
    // Object heights (for distance calculation)
    public static final double CORAL_HEIGHT_METERS = Units.inchesToMeters(6.0);  // ✅ Measure actual coral
    public static final double ALGAE_HEIGHT_METERS = Units.inchesToMeters(3.0);  // ✅ Measure actual algae
    
    // Object detection class IDs (from trained model)
    public static final int CORAL_CLASS_ID = 0;   // ✅ Usually 0 for first class
    public static final int ALGAE_CLASS_ID = 1;   // ✅ 1 for second class (if training both)
}
    
    /** Enable/disable flags */
    public static final class SubsystemFlags {
        public static final boolean ENABLED = true;  // Master enable
        public static final boolean DEBUG_TELEMETRY = true;  // SmartDashboard output
        public static final boolean CONSOLE_OUTPUT = false;  // Console logging
    }
    
    // ==================== OBJECT TYPES ====================
    
    public enum GamePiece {
        CORAL("Coral", DetectionConstants.CORAL_PIPELINE, DetectionConstants.CORAL_HEIGHT_METERS),
        ALGAE("Algae", DetectionConstants.ALGAE_PIPELINE, DetectionConstants.ALGAE_HEIGHT_METERS);
        
        private final String name;
        private final int pipelineIndex;
        private final double heightMeters;
        
        GamePiece(String name, int pipelineIndex, double heightMeters) {
            this.name = name;
            this.pipelineIndex = pipelineIndex;
            this.heightMeters = heightMeters;
        }
        
        public String getName() { return name; }
        public int getPipelineIndex() { return pipelineIndex; }
        public double getHeightMeters() { return heightMeters; }
    }
    
    // ==================== INSTANCE VARIABLES ====================
    
    private final PhotonCamera camera;
    private PhotonPipelineResult latestResult;
    private GamePiece currentTargetType = GamePiece.CORAL;
    private boolean hasTarget = false;
    private PhotonTrackedTarget bestTarget = null;
    
    // Cached calculations
    private double distanceToTarget = 0.0;
    private double angleToTarget = 0.0;
    private Pose2d estimatedTargetPose = null;
    
    // ==================== CONSTRUCTOR ====================
    
    public PhotonVisionSubsystem() {
        if (!SubsystemFlags.ENABLED) {
            camera = null;
            System.out.println("⚠️ PhotonVision subsystem DISABLED");
            return;
        }
        
        camera = new PhotonCamera(CameraConstants.CAMERA_NAME);
        
        // Set initial pipeline
        setPipeline(currentTargetType);
        
        System.out.println("✅ PhotonVision subsystem initialized");
        System.out.println("   Camera: " + CameraConstants.CAMERA_NAME);
        System.out.println("   IP: " + CameraConstants.CAMERA_IP);
        System.out.println("   Default target: " + currentTargetType.getName());
    }
    
    // ==================== PIPELINE MANAGEMENT ====================
    
    /**
     * Switch detection pipeline (Coral, Algae, AprilTags, etc.)
     */
    public void setPipeline(GamePiece targetType) {
        if (camera == null) return;
        
        currentTargetType = targetType;
        camera.setPipelineIndex(targetType.getPipelineIndex());
        
        if (SubsystemFlags.CONSOLE_OUTPUT) {
            System.out.println("📷 Switched to " + targetType.getName() + " detection");
        }
    }
    
    public void setCoralDetection() { setPipeline(GamePiece.CORAL); }
    public void setAlgaeDetection() { setPipeline(GamePiece.ALGAE); }
    
    public GamePiece getCurrentTargetType() { return currentTargetType; }
    
    // ==================== TARGET DETECTION ====================
    
    /**
     * Check if we see any valid targets
     */
    public boolean hasValidTarget() {
        if (!hasTarget || bestTarget == null) return false;
        
        // Filter by confidence (for object detection models)
        // Note: PhotonTrackedTarget doesn't have getConfidence() yet in some versions
        // This is a placeholder - check PhotonLib API for your version
        
        // Filter by area
        if (bestTarget.getArea() < DetectionConstants.MIN_TARGET_AREA) return false;
        
        // Filter by distance
        if (distanceToTarget > DetectionConstants.MAX_DETECTION_RANGE_METERS) return false;
        
        return true;
    }
    
    /**
     * Get the best (closest/largest) detected target
     */
    public Optional<PhotonTrackedTarget> getBestTarget() {
        return hasValidTarget() ? Optional.of(bestTarget) : Optional.empty();
    }
    
    /**
     * Get all detected targets (for multi-object tracking)
     */
    public List<PhotonTrackedTarget> getAllTargets() {
        if (latestResult == null || !hasTarget) {
            return List.of();
        }
        return latestResult.getTargets();
    }

    /**
     * Get confidence/probability of best target detection (0-1)
     * Only works with object detection pipelines
     */
    public double getTargetConfidence() {
        if (!hasValidTarget()) return 0.0;
        
        // For object detection models, PhotonVision may provide confidence
        // Check your PhotonLib version for exact API
        // This is a placeholder - update based on actual API
        
        // Fallback: use area as proxy for confidence
        return Math.min(1.0, bestTarget.getArea() / 10.0);
    }
    
    /**
     * Number of targets detected
     */
    public int getTargetCount() {
        return getAllTargets().size();
    }
    
    // ==================== TARGET MEASUREMENTS ====================
    
    /**
     * Get horizontal angle to target (degrees)
     * Positive = target is to the right
     * Negative = target is to the left
     */
    public double getYawToTarget() {
        return angleToTarget;
    }
    
    /**
     * Get vertical angle to target (degrees)
     */
    public double getPitchToTarget() {
        if (!hasValidTarget()) return 0.0;
        return bestTarget.getPitch();
    }
    
    /**
     * Get estimated distance to target (meters)
     */
    public double getDistanceToTarget() {
        return distanceToTarget;
    }
    
    /**
     * Get target area as % of image (0-100)
     */
    public double getTargetArea() {
        if (!hasValidTarget()) return 0.0;
        return bestTarget.getArea();
    }
    
    /**
     * Get target's skew/rotation (degrees)
     */
    public double getTargetSkew() {
        if (!hasValidTarget()) return 0.0;
        return bestTarget.getSkew();
    }
    
    // ==================== FIELD-RELATIVE POSITIONING ====================
    
    /**
     * Get estimated field position of detected object
     * Returns null if no valid target or robot pose unavailable
     */
    public Pose2d getEstimatedTargetPose() {
        return estimatedTargetPose;
    }
    
    /**
     * Get translation vector from robot to target (robot-relative)
     */
    public Optional<Translation2d> getTranslationToTarget() {
        if (!hasValidTarget()) return Optional.empty();
        
        // Calculate X (forward) and Y (left) distances
        double angleRad = Math.toRadians(angleToTarget);
        double targetX = distanceToTarget * Math.cos(angleRad);
        double targetY = distanceToTarget * Math.sin(angleRad);
        
        return Optional.of(new Translation2d(targetX, targetY));
    }
    
    // ==================== HELPER METHODS ====================
    
    /**
     * Calculate distance to target using camera pitch and target height
     */
    private double calculateDistance(PhotonTrackedTarget target) {
        if (target == null) return 0.0;
        
        // Use PhotonUtils for trigonometric distance calculation
        double distance = PhotonUtils.calculateDistanceToTargetMeters(
            CameraConstants.CAMERA_HEIGHT_METERS,
            currentTargetType.getHeightMeters(),
            Math.toRadians(CameraConstants.CAMERA_PITCH_DEGREES),
            Math.toRadians(target.getPitch())
        );
        
        return distance;
    }
    
    /**
     * Calculate field-relative pose of detected object
     */
    private void updateTargetPose() {
        if (!hasValidTarget()) {
            estimatedTargetPose = null;
            return;
        }
        
        // Get robot pose from drivetrain
        Pose2d robotPose = RobotContainer.driveSubsystem.getPose();
        
        // Calculate target position relative to robot
        double angleRad = Math.toRadians(robotPose.getRotation().getDegrees() + angleToTarget);
        double targetX = robotPose.getX() + distanceToTarget * Math.cos(angleRad);
        double targetY = robotPose.getY() + distanceToTarget * Math.sin(angleRad);
        
        estimatedTargetPose = new Pose2d(targetX, targetY, new Rotation2d());
    }
    
    // ==================== TELEMETRY ====================
    
    private void updateTelemetry() {
        if (!SubsystemFlags.DEBUG_TELEMETRY) return;
        
        // Basic status
        SmartDashboard.putString("PV Target Type", currentTargetType.getName());
        SmartDashboard.putBoolean("PV Has Target", hasValidTarget());
        SmartDashboard.putNumber("PV Target Count", getTargetCount());
        SmartDashboard.putBoolean("PV Camera Connected", camera != null && camera.isConnected());
        
        // Target data
        if (hasValidTarget()) {
            SmartDashboard.putNumber("PV Yaw (deg)", angleToTarget);
            SmartDashboard.putNumber("PV Pitch (deg)", getPitchToTarget());
            SmartDashboard.putNumber("PV Distance (m)", distanceToTarget);
            SmartDashboard.putNumber("PV Area (%)", getTargetArea());
            SmartDashboard.putNumber("PV Confidence", getTargetConfidence());  // ✅ ADDED
            SmartDashboard.putNumber("PV Skew (deg)", getTargetSkew());
            
            // Field position
            if (estimatedTargetPose != null) {
                SmartDashboard.putNumber("PV Target X", estimatedTargetPose.getX());
                SmartDashboard.putNumber("PV Target Y", estimatedTargetPose.getY());
            }
        } else {
            SmartDashboard.putNumber("PV Yaw (deg)", 0);
            SmartDashboard.putNumber("PV Pitch (deg)", 0);
            SmartDashboard.putNumber("PV Distance (m)", 0);
            SmartDashboard.putNumber("PV Area (%)", 0);
            SmartDashboard.putNumber("PV Confidence", 0);  // ✅ ADDED
        }
    }
    
    // ==================== PERIODIC ====================
    
    @Override
    public void periodic() {
        if (camera == null) return;
        
        // Get latest camera results
        latestResult = camera.getLatestResult();
        hasTarget = latestResult.hasTargets();
        
        if (hasTarget) {
            // Get best target (largest area / closest)
            bestTarget = latestResult.getBestTarget();
            
            // Calculate measurements
            angleToTarget = bestTarget.getYaw();
            distanceToTarget = calculateDistance(bestTarget);
            
            // Update field-relative position
            updateTargetPose();
        } else {
            bestTarget = null;
            distanceToTarget = 0.0;
            angleToTarget = 0.0;
            estimatedTargetPose = null;
        }
        
        // Update dashboard
        updateTelemetry();
    }
    
    // ==================== UTILITY METHODS ====================
    
    /**
     * Check if camera is connected and operational
     */
    public boolean isCameraConnected() {
        return camera != null && camera.isConnected();
    }
    
    /**
     * Get camera latency (ms)
     
    public double getLatency() {
        if (latestResult == null) return 0.0;
        return latestResult.getLatencySeconds() * 1000.0;
    }
        */
    
    /**
     * Get timestamp of latest result
     */
    public double getTimestamp() {
        if (latestResult == null) return 0.0;
        return latestResult.getTimestampSeconds();
    }
}