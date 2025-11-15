package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants.SwerveConstants;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToTargetCommand extends Command {
    
    private final DriveSubsystem driveSubsystem;
    private final PhotonVisionSubsystem visionSubsystem;
    private final PIDController rotationController;
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(8.0); // 8 rad/s² max accel
    
    private final SwerveRequest.FieldCentric turnRequest = new SwerveRequest.FieldCentric()
        .withDeadband(SwerveConstants.MaxSpeed * SwerveConstants.DeadbandRatioLinear)
        .withRotationalDeadband(SwerveConstants.MaxAngularRate * SwerveConstants.DeadbandRatioAngular)
        .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
    
    // ✅ TUNED CONSTANTS (adjust these for your robot)
    private static final double kP = 0.12;   // Proportional gain
    private static final double kI = 0.02;   // Integral gain (eliminates steady-state error)
    private static final double kD = 0.015;  // Derivative gain (damping)
    
    private static final double ANGLE_TOLERANCE = 0.5;      // Degrees (very tight)
    private static final double MAX_ROTATION_SPEED = 3.0;   // rad/s (smoother than 4.0)
    private static final double MIN_ROTATION_SPEED = 0.15;  // Overcome static friction
    
    public TurnToTargetCommand(DriveSubsystem driveSubsystem, PhotonVisionSubsystem visionSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        
        rotationController = new PIDController(kP, kI, kD);
        rotationController.setTolerance(ANGLE_TOLERANCE);
        rotationController.enableContinuousInput(-180, 180);
        
        addRequirements(driveSubsystem, visionSubsystem);
    }
    
    @Override
    public void initialize() {
        System.out.println("🎯 TurnToTarget: Started");
        rotationController.reset();
        rotationLimiter.reset(0);
        SmartDashboard.putBoolean("TurnToTarget/Active", true);
        
        // Publish tuning values
        SmartDashboard.putNumber("TurnToTarget/Tune/kP", kP);
        SmartDashboard.putNumber("TurnToTarget/Tune/kI", kI);
        SmartDashboard.putNumber("TurnToTarget/Tune/kD", kD);
    }
    
    @Override
    public void execute() {
        if (!visionSubsystem.hasValidTarget()) {
            driveSubsystem.setControl(turnRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            SmartDashboard.putBoolean("TurnToTarget/HasTarget", false);
            return;
        }
        
        SmartDashboard.putBoolean("TurnToTarget/HasTarget", true);
        
        double angleError = visionSubsystem.getYawToTarget();
        
        // Calculate PID output
        double rotationSpeed = rotationController.calculate(angleError, 0);
        
        // Apply minimum speed to overcome friction
        if (Math.abs(angleError) > ANGLE_TOLERANCE && Math.abs(rotationSpeed) < MIN_ROTATION_SPEED) {
            rotationSpeed = Math.copySign(MIN_ROTATION_SPEED, rotationSpeed);
        }
        
        // Clamp to max speed
        rotationSpeed = Math.max(-MAX_ROTATION_SPEED, Math.min(MAX_ROTATION_SPEED, rotationSpeed));
        
        // Apply slew rate limiting for smooth acceleration
        rotationSpeed = rotationLimiter.calculate(rotationSpeed);
        
        driveSubsystem.setControl(
            turnRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(rotationSpeed)
        );
        
        // Telemetry
        SmartDashboard.putNumber("TurnToTarget/AngleError", angleError);
        SmartDashboard.putNumber("TurnToTarget/RotationSpeed", rotationSpeed);
        SmartDashboard.putBoolean("TurnToTarget/OnTarget", Math.abs(angleError) < ANGLE_TOLERANCE);
        SmartDashboard.putNumber("TurnToTarget/Distance", visionSubsystem.getDistanceToTarget());
        
        // Reduced console spam
        if (Math.abs(angleError) > 0.5) {
            System.out.printf("🎯 Yaw: %.2f° | Speed: %.2f rad/s | OnTarget: %b\n", 
                angleError, rotationSpeed, Math.abs(angleError) < ANGLE_TOLERANCE);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setControl(turnRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        SmartDashboard.putBoolean("TurnToTarget/Active", false);
        System.out.println("🎯 TurnToTarget: " + (interrupted ? "Interrupted" : "Finished"));
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}