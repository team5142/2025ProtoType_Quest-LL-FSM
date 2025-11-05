package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class TwoMeterTest extends Command {
    private final double TARGET_DISTANCE = 2.0; // meters
    private double startX;
    private double startY;
    private Rotation2d startHeading;
    private int loopCounter = 0;

    public TwoMeterTest() {
        addRequirements(RobotContainer.driveSubsystem);
    }
    @Override
    public void initialize() {
        // Record starting position and heading
        var currentPose = RobotContainer.driveSubsystem.getState().Pose;
        startX = currentPose.getX();
        startY = currentPose.getY();
        startHeading = currentPose.getRotation();
        loopCounter = 0;
        
        System.out.println("=== Starting 2-meter test ===");
        System.out.println("Start X: " + startX);
        System.out.println("Start Y: " + startY);
        System.out.println("Start Heading: " + startHeading.getDegrees() + "°");
    }

    @Override
    public void execute() {
        Rotation2d currentHeading = RobotContainer.driveSubsystem.getState().Pose.getRotation();
        double headingError = currentHeading.minus(startHeading).getRadians();
        double rotationCorrection = -headingError * 2.0;
        
        RobotContainer.driveSubsystem.drive(-1.0, 0.0, rotationCorrection);
        
        // Print every 25 loops (~0.5 seconds at 50Hz)
        loopCounter++;
        if (loopCounter % 25 == 0) {
            System.out.printf("Heading error: %.2f°, Correction: %.2f rad/s%n", 
                Math.toDegrees(headingError), rotationCorrection);
        }
    }

    @Override
    public boolean isFinished() {
        double currentX = RobotContainer.driveSubsystem.getState().Pose.getX();
        double distanceTraveled = Math.abs(currentX - startX);
        
        if (distanceTraveled >= TARGET_DISTANCE) {
            System.out.println("*** Target distance reached!");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop robot
        RobotContainer.driveSubsystem.drive(0, 0, 0);
        
        // Get final measurements
        var finalPose = RobotContainer.driveSubsystem.getState().Pose;
        double finalX = finalPose.getX();
        double finalY = finalPose.getY();
        Rotation2d finalHeading = finalPose.getRotation();
        
        // Calculate deltas
        double distanceX = finalX - startX;
        double driftY = finalY - startY;
        double headingDrift = finalHeading.minus(startHeading).getDegrees();
        
        // Print results
        System.out.println("=== Test Complete ===");
        System.out.println(String.format("Distance traveled (X): %.3f meters (%.2f inches)", 
            distanceX, distanceX * 39.37));
        System.out.println(String.format("Lateral drift (Y): %.3f meters (%.2f inches)", 
            driftY, driftY * 39.37));
        System.out.println(String.format("Heading drift: %.2f°", headingDrift));
        System.out.println("Expected: 2.00m forward, 0.00m lateral, 0° heading");
        
        if (interrupted) {
            System.out.println("*** Test was interrupted!");
        }
    }
}