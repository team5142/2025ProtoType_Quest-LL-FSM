package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class LShapeTest extends Command {
    // Test parameters
    private static final double FORWARD_DISTANCE = 3.0; // 3 meters forward
    private static final double STRAFE_DISTANCE = 1.5;  // 1.5 meters left
    private static final double DRIVE_SPEED = 1.0;      // 1 m/s
    private static final double HEADING_KP = 2.0;       // Heading lock P gain
    
    // Test phases
    private enum Phase {
        FORWARD,      // Phase 1: Drive forward 3m
        ROTATE,       // Phase 2: Rotate to 90°
        STRAFE,       // Phase 3: Strafe left 1.5m
        COMPLETE      // Phase 4: Done
    }
    
    private Phase currentPhase;
    private double startX, startY;
    private Rotation2d startHeading;
    private double phaseStartX, phaseStartY;
    private int rotationLoops = 0;

    public LShapeTest() {
        addRequirements(RobotContainer.driveSubsystem);
    }

    @Override
    public void initialize() {
        // Record starting position
        var startPose = RobotContainer.driveSubsystem.getState().Pose;
        startX = startPose.getX();
        startY = startPose.getY();
        startHeading = startPose.getRotation();
        
        // Start with forward phase
        currentPhase = Phase.FORWARD;
        rotationLoops = 0;
        phaseStartX = startX;
        phaseStartY = startY;
        
        System.out.println("=== L-Shape Test Started ===");
        System.out.println("Start: (" + startX + ", " + startY + ") @ " + startHeading.getDegrees() + "°");
        System.out.println("Phase 1: Drive forward 3m");
    }

    @Override
    public void execute() {
        var currentPose = RobotContainer.driveSubsystem.getState().Pose;
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        Rotation2d currentHeading = currentPose.getRotation();
        
        switch (currentPhase) {
            case FORWARD:
                // Drive forward with heading lock
                double forwardDist = Math.abs(currentX - phaseStartX);
                
                if (forwardDist >= FORWARD_DISTANCE) {
                    // Phase complete - transition to rotation
                    currentPhase = Phase.ROTATE;
                    phaseStartX = currentX;
                    phaseStartY = currentY;
                    System.out.println("Phase 1 complete. Actual distance: " + forwardDist + "m");
                    System.out.println("Phase 2: Rotate to 90°");
                } else {
                    // Drive forward with heading lock
                    double headingError = currentHeading.minus(startHeading).getRadians();
                    double rotationCorrection = -headingError * HEADING_KP;
                    RobotContainer.driveSubsystem.drive(-DRIVE_SPEED, 0.0, rotationCorrection);
                }
                break;
                
            case ROTATE:
    rotationLoops++;  // ← ADD THIS
    
    Rotation2d targetHeading = startHeading.plus(Rotation2d.fromDegrees(90));
    double rotationError = currentHeading.minus(targetHeading).getDegrees();
    
    // Add timeout check (3 seconds at 50Hz = 150 loops)
    if (Math.abs(rotationError) < 2.0 || rotationLoops > 150) {
        if (rotationLoops > 150) {
            System.out.println("*** Rotation timeout! Final error: " + rotationError + "°");
        }
        currentPhase = Phase.STRAFE;
        phaseStartX = currentX;
        phaseStartY = currentY;
        rotationLoops = 0;  // ← RESET COUNTER
        System.out.println("Phase 2 complete. Final heading: " + currentHeading.getDegrees() + "°");
        System.out.println("Phase 3: Strafe left 1.5m");
    } else {
        // Continue rotating
        double omega = -rotationError * 0.15;  // ← INCREASE from 0.03
        omega = Math.max(-4.0, Math.min(4.0, omega)); // ← INCREASE from ±2.0
        RobotContainer.driveSubsystem.drive(0.0, 0.0, omega);
        
        // Less frequent debug (every 0.5 seconds)
        if (rotationLoops % 25 == 0) {
            System.out.printf("Rotating: Current=%.1f°, Target=%.1f°, Error=%.1f°, Omega=%.2f%n",
                currentHeading.getDegrees(), targetHeading.getDegrees(), rotationError, omega);
        }
    }
    break;
                
            case STRAFE:
                // Strafe left with heading lock at 90°
                double strafeDist = Math.abs(currentY - phaseStartY);
                Rotation2d strafeTargetHeading = startHeading.plus(Rotation2d.fromDegrees(90));
                
                if (strafeDist >= STRAFE_DISTANCE) {
                    // Phase complete
                    currentPhase = Phase.COMPLETE;
                    System.out.println("Phase 3 complete. Actual distance: " + strafeDist + "m");
                } else {
                    // Strafe left with heading lock
                    double headingError = currentHeading.minus(strafeTargetHeading).getRadians();
                    double rotationCorrection = -headingError * HEADING_KP;
                    RobotContainer.driveSubsystem.drive(0.0, -DRIVE_SPEED, rotationCorrection);
                }
                break;
                
            case COMPLETE:
                // Stop
                RobotContainer.driveSubsystem.drive(0.0, 0.0, 0.0);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return currentPhase == Phase.COMPLETE;
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.driveSubsystem.drive(0, 0, 0);
        
        var finalPose = RobotContainer.driveSubsystem.getState().Pose;
        double finalX = finalPose.getX();
        double finalY = finalPose.getY();
        Rotation2d finalHeading = finalPose.getRotation();
        
        double totalDistX = finalX - startX;
        double totalDistY = finalY - startY;
        double headingChange = finalHeading.minus(startHeading).getDegrees();
        
        System.out.println("=== L-Shape Test Complete ===");
        System.out.println(String.format("Forward distance: %.3fm (expected 3.00m)", Math.abs(totalDistX)));
        System.out.println(String.format("Strafe distance: %.3fm (expected 1.50m)", Math.abs(totalDistY)));
        System.out.println(String.format("Final heading: %.1f° (expected 90°)", headingChange));
        System.out.println("Start: (" + startX + ", " + startY + ")");
        System.out.println("End: (" + finalX + ", " + finalY + ")");
        
        if (interrupted) {
            System.out.println("*** Test was interrupted!");
        }
    }
}