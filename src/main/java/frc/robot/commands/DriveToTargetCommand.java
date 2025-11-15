package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.io.IOException;

public class DriveToTargetCommand extends Command {

    private final DriveSubsystem driveSubsystem;
    private final PhotonVisionSubsystem visionSubsystem;
    private final PIDController rotationController;
    private final PIDController forwardController;
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(12.0);
    private final SlewRateLimiter forwardLimiter = new SlewRateLimiter(6.0);

    private PrintWriter logFile;

    // Safety
    private static final double SAFETY_SPEED_MULTIPLIER = .8;
    private static final double EMERGENCY_STOP_DISTANCE = 0.3;

    // PID constants
    private static final double ROTATION_kP = 0.12;
    private static final double ROTATION_kI = 0.02;
    private static final double ROTATION_kD = 0.015;

    private static final double FORWARD_kP = 2.5;
    private static final double FORWARD_kI = 0.0;
    private static final double FORWARD_kD = 0.3;

    // Target settings
    private static final double TARGET_DISTANCE = 0.5;
    private static final double DISTANCE_TOLERANCE = 0.1;
    private static final double ANGLE_TOLERANCE = 2.0;

    private static final double MAX_ROTATION_SPEED = 3.0;
    private static final double MAX_FORWARD_SPEED = 2.0;
    private static final double MIN_FORWARD_SPEED = 0.1;

    private static final double ALIGNMENT_THRESHOLD = 35.0;

    private int executeCount = 0;

    // ✅ CONSTRUCTOR: Initialize PID controllers here
    public DriveToTargetCommand(DriveSubsystem driveSubsystem, PhotonVisionSubsystem visionSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;

        // ✅ CREATE THE PID CONTROLLERS
        rotationController = new PIDController(ROTATION_kP, ROTATION_kI, ROTATION_kD);
        rotationController.setTolerance(ANGLE_TOLERANCE);
        rotationController.enableContinuousInput(-180, 180);

        forwardController = new PIDController(FORWARD_kP, FORWARD_kI, FORWARD_kD);
        forwardController.setTolerance(DISTANCE_TOLERANCE);
        // DON'T set setpoint - we'll use distance error directly

        addRequirements(driveSubsystem, visionSubsystem);
    }

    private void log(String message) {
        if (logFile != null) {
            logFile.println(message);
            logFile.flush();
        }
        System.out.println(message);
    }

    @Override
    public void initialize() {
        executeCount = 0;

        try {
            logFile = new PrintWriter(new FileWriter("/home/lvuser/DriveToTarget.log", false));
            log("========================================");
            log("DriveToTarget INITIALIZE");
            log("Time: " + System.currentTimeMillis());
            log("========================================");
        } catch (IOException e) {
            System.err.println("Failed to create log file: " + e.getMessage());
        }

        rotationController.reset();
        forwardController.reset();
        rotationLimiter.reset(0);
        forwardLimiter.reset(0);
        SmartDashboard.putBoolean("DriveToTarget/Active", true);
    }

    @Override
    public void execute() {
        executeCount++;

        if (!visionSubsystem.hasValidTarget()) {
            driveSubsystem.driveVision(0, 0, 0);
            SmartDashboard.putBoolean("DriveToTarget/HasTarget", false);
            if (executeCount % 20 == 1) {
                log("Execute #" + executeCount + " - NO TARGET");
            }
            return;
        }

        SmartDashboard.putBoolean("DriveToTarget/HasTarget", true);

        double angleError = visionSubsystem.getYawToTarget();
        double currentDistance = visionSubsystem.getDistanceToTarget();
        double absDistance = Math.abs(currentDistance);

        if (absDistance < EMERGENCY_STOP_DISTANCE) {
            driveSubsystem.driveVision(0, 0, 0);
            SmartDashboard.putBoolean("DriveToTarget/EmergencyStop", true);
            if (executeCount % 20 == 1) {
                log(String.format("Execute #%d - EMERGENCY STOP (%.3fm)", executeCount, absDistance));
            }
            return;
        }
        SmartDashboard.putBoolean("DriveToTarget/EmergencyStop", false);

        // ===== ROTATION =====
        double rotationSpeed = rotationController.calculate(angleError, 0);
        if (Math.abs(angleError) < ANGLE_TOLERANCE) {
            rotationSpeed = 0;
        }
        rotationSpeed = Math.max(-MAX_ROTATION_SPEED, Math.min(MAX_ROTATION_SPEED, rotationSpeed));
        rotationSpeed = rotationLimiter.calculate(rotationSpeed);

        // ===== FORWARD DRIVE =====
        double distanceError = TARGET_DISTANCE - absDistance;
        double forwardSpeed = forwardController.calculate(distanceError, 0);

        //if (Math.abs(distanceError) > DISTANCE_TOLERANCE && Math.abs(forwardSpeed) < MIN_FORWARD_SPEED) {
        //    forwardSpeed = Math.copySign(MIN_FORWARD_SPEED, forwardSpeed);
        //}

        forwardSpeed = Math.max(-MAX_FORWARD_SPEED, Math.min(MAX_FORWARD_SPEED, forwardSpeed));
        forwardSpeed = forwardLimiter.calculate(forwardSpeed);

        if (Math.abs(distanceError) < DISTANCE_TOLERANCE) {
            forwardSpeed = 0;
        }

        // Apply safety multiplier
        rotationSpeed *= SAFETY_SPEED_MULTIPLIER;
        forwardSpeed *= SAFETY_SPEED_MULTIPLIER;

        // Convert to field coordinates
        double robotHeading = driveSubsystem.getPose().getRotation().getRadians();
        double fieldX = forwardSpeed * Math.cos(robotHeading);
        double fieldY = forwardSpeed * Math.sin(robotHeading);

        if (executeCount % 10 == 0) {
            log(String.format("Execute #%d:", executeCount));
            log(String.format("  Angle: %.2f°  |  Dist: %.3fm (raw: %.3f)", angleError, absDistance, currentDistance));
            log(String.format("  Rotation: %.3f rad/s  |  Forward: %.3f m/s", rotationSpeed, forwardSpeed));
            log(String.format("  Robot heading: %.1f°", Math.toDegrees(robotHeading)));
            log(String.format("  Field X: %.3f  |  Field Y: %.3f", fieldX, fieldY));
            log(String.format("  CALLING driveSubsystem.driveVision(%.3f, %.3f, %.3f)", fieldX, fieldY, rotationSpeed));
            log("---");
        }

        driveSubsystem.driveVision(fieldX, fieldY, rotationSpeed);

        SmartDashboard.putNumber("DriveToTarget/AngleError", angleError);
        SmartDashboard.putNumber("DriveToTarget/Distance", absDistance);
        SmartDashboard.putNumber("DriveToTarget/FieldX", fieldX);
        SmartDashboard.putNumber("DriveToTarget/FieldY", fieldY);
        SmartDashboard.putNumber("DriveToTarget/RotationSpeed", rotationSpeed);
        SmartDashboard.putNumber("DriveToTarget/ExecuteCount", executeCount);
    }

    @Override
    public void end(boolean interrupted) {
        log("========================================");
        log("DriveToTarget END (interrupted=" + interrupted + ")");
        log("Total executes: " + executeCount);
        log("========================================");

        if (logFile != null) {
            logFile.close();
        }

        driveSubsystem.driveVision(0, 0, 0);
        SmartDashboard.putBoolean("DriveToTarget/Active", false);
    }

    @Override
    public boolean isFinished() {
        if (!visionSubsystem.hasValidTarget()) {
            return false;
        }
        
        double angleError = Math.abs(visionSubsystem.getYawToTarget());
        double distanceError = Math.abs(Math.abs(visionSubsystem.getDistanceToTarget()) - TARGET_DISTANCE);
        
        // ✅ Finish when both aligned AND at target distance
        return angleError < ANGLE_TOLERANCE && distanceError < DISTANCE_TOLERANCE;
    }
}