// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.OperatorConstants.SwerveConstants;
import frc.robot.OdometryUpdates.LLAprilTagConstants.LLVisionConstants.LLCamera;
import frc.robot.OdometryUpdates.LLAprilTagSubsystem;
import frc.robot.OdometryUpdates.OdometryUpdatesSubsystem;
import frc.robot.OdometryUpdates.QuestNavSubsystem;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.DriveToTargetCommand;
import frc.robot.commands.LShapeTest;
import frc.robot.commands.OneMeterForwardPPCommand;
import frc.robot.commands.ReturnTestPPCommand;
import frc.robot.commands.StopRobot;
import frc.robot.commands.TwoMeterTest;
import frc.robot.lib.LimelightHelpers;
import frc.robot.lib.TrajectoryHelper;
import frc.robot.subsystems.DriveSubsystem;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.commands.TurnToTargetCommand;
import frc.robot.commands.DriveToTargetCommand;


import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem.GamePiece;

public class RobotContainer {

  // kSpeedAt12Volts desired top speed
  // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  // Use open-loop control for drive motors
  private final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

  private final CommandXboxController xboxDriveController = new CommandXboxController(0);
  public static boolean isAllianceRed = false;
  public static boolean isReversingControllerAndIMUForRed = true;

  public static final DriveSubsystem driveSubsystem = DriveSubsystem.createDrivetrain();
  public static QuestNavSubsystem questNavSubsystem = new QuestNavSubsystem();
  public static LLAprilTagSubsystem llAprilTagSubsystem = new LLAprilTagSubsystem();
  public static OdometryUpdatesSubsystem odometryUpdateSubsystem = new OdometryUpdatesSubsystem();
  public static PhotonVisionSubsystem photonVisionSubsystem = new PhotonVisionSubsystem();

  // Slew rate limiters for smooth acceleration
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(3); // 3 m/s² acceleration
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(3); // 3 m/s² acceleration
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(6); // 6 rad/s² rotational acceleration

  private double lastXCommand = 0;
  private double lastYCommand = 0;
  private double lastRotCommand = 0;

  public void resetSlewRateLimiters() {
    xLimiter.reset(0);
    yLimiter.reset(0);
    rotLimiter.reset(0);
  }

  public RobotContainer() {
    configureBindings();
    configureVisionBindings();
    testTrajectory();
    setYaws();

    /*
     * driveSubsystem.setDefaultCommand(
     * new DriveManuallyCommand(
     * () -> getDriverForwardBackward(),
     * () -> getDriverLeftRight(),
     * () -> getDriverOmegaAxis()));
     */
    // ✅ NEW - Direct drive command
    driveSubsystem.setDefaultCommand(
        driveSubsystem.run(() -> {
          // Get raw joystick values
          /*
           * double forwardBack = -xboxDriveController.getLeftY();
           * double leftRight = -xboxDriveController.getLeftX();
           * double rotation = -xboxDriveController.getRightX();
           * 
           * // Apply deadband
           * if (Math.abs(forwardBack) < 0.1) forwardBack = 0;
           * if (Math.abs(leftRight) < 0.1) leftRight = 0;
           * if (Math.abs(rotation) < 0.1) rotation = 0;
           * 
           * // Scale to max speeds (50% for testing)
           * double xVel = forwardBack * SwerveConstants.MaxSpeed * 0.5;
           * double yVel = leftRight * SwerveConstants.MaxSpeed * 0.5;
           * double omegaVel = rotation * SwerveConstants.MaxAngularRate * 0.5;
           * 
           * // Drive!
           * driveSubsystem.drive(xVel, yVel, omegaVel);
           */
          driveSubsystem.drive(getDriverForwardBackward(), getDriverLeftRight(), getDriverOmegaAxis());
        }));
    FollowPathCommand.warmupCommand().schedule();
  }

  // Driver controls - properly scaled to max speeds
  private double getDriverForwardBackward() {
    double rawValue = -xboxDriveController.getLeftY();

    // Aggressive stop on joystick release
    if (Math.abs(rawValue) < 0.1) {
      yLimiter.reset(0);
      lastYCommand = 0;
      return 0;
    }

    // Scale to 50% speed for testing
    rawValue *= SwerveConstants.MaxSpeed * 0.5;
    lastYCommand = yLimiter.calculate(rawValue);
    return lastYCommand;
  }

  private double getDriverLeftRight() {
    double rawValue = -xboxDriveController.getLeftX();

    // Aggressive stop on joystick release
    if (Math.abs(rawValue) < 0.1) {
      xLimiter.reset(0);
      lastXCommand = 0;
      return 0;
    }

    // Scale to 50% speed for testing
    rawValue *= SwerveConstants.MaxSpeed * 0.5;
    lastXCommand = xLimiter.calculate(rawValue);
    return lastXCommand;
  }

  private double getDriverOmegaAxis() {
    double rawValue = -xboxDriveController.getRightX();

    // Aggressive stop on joystick release
    if (Math.abs(rawValue) < 0.1) {
      rotLimiter.reset(0);
      lastRotCommand = 0;
      return 0;
    }

    // Scale to 50% speed for testing
    rawValue *= SwerveConstants.MaxAngularRate * 0.5;
    lastRotCommand = rotLimiter.calculate(rawValue);
    return lastRotCommand;
  }

  private void configureBindings() {
    // Idle while the robot is disabled. This ensures the configured
    RobotModeTriggers.disabled()
        .whileTrue(driveSubsystem.applyRequest(() -> driveSubsystem.getIdle()).ignoringDisable(true));
    driveSubsystem.registerTelemetry(logger::telemeterize);
  }

  private void configureVisionBindings() {
    // D-Pad UP: Switch to Algae detection
    xboxDriveController.povUp().onTrue(
        new InstantCommand(() -> photonVisionSubsystem.setAlgaeDetection()));
    
    // D-Pad RIGHT: Switch to Coral detection
    xboxDriveController.povRight().onTrue(
        new InstantCommand(() -> photonVisionSubsystem.setCoralDetection()));
    
    // ✅ A button: Turn to face target (HOLD to track)
    xboxDriveController.a().whileTrue(
        new TurnToTargetCommand(driveSubsystem, photonVisionSubsystem)
    );
    
    // ✅ B button: Drive to target (HOLD to approach)
    xboxDriveController.b().whileTrue(
        new DriveToTargetCommand(driveSubsystem, photonVisionSubsystem)
    );
    
    // X button: Print detection info (moved from B)
    xboxDriveController.x().onTrue(
        new InstantCommand(() -> {
            if (photonVisionSubsystem.hasValidTarget()) {
                System.out.println("🎯 Detected " + photonVisionSubsystem.getCurrentTargetType().getName());
                System.out.println("   Distance: " + photonVisionSubsystem.getDistanceToTarget() + " m");
                System.out.println("   Angle: " + photonVisionSubsystem.getYawToTarget() + "°");
            } else {
                System.out.println("❌ No target detected");
            }
        }));
}

  private void testTrajectory() {
    // D-Pad DOWN: Detailed rotation debug (SmartDashboard + Console)
    xboxDriveController.povDown().onTrue(
        new InstantCommand(() -> {
          // Raw gyro value
          double gyroYaw = driveSubsystem.getPigeon2().getYaw().getValueAsDouble();

          // Pose rotation
          Rotation2d poseRotation = driveSubsystem.getPose().getRotation();
          Pose2d fullPose = driveSubsystem.getPose();

          // Calculate difference
          double difference = Math.abs(gyroYaw - poseRotation.getDegrees());

          // === SMARTDASHBOARD OUTPUT (no slashes) ===
          SmartDashboard.putNumber("Debug Gyro Yaw", gyroYaw);
          SmartDashboard.putNumber("Debug Pose Rotation", poseRotation.getDegrees());
          SmartDashboard.putNumber("Debug Pose Radians", poseRotation.getRadians());
          SmartDashboard.putNumber("Debug Difference", difference);
          SmartDashboard.putNumber("Debug Pose X", fullPose.getX());
          SmartDashboard.putNumber("Debug Pose Y", fullPose.getY());

          // Create summary string for easy copying
          String debugSummary = String.format(
              "Gyro: %.2f° | Pose: %.2f° | Diff: %.2f° | X: %.3f | Y: %.3f",
              gyroYaw,
              poseRotation.getDegrees(),
              difference,
              fullPose.getX(),
              fullPose.getY());
          SmartDashboard.putString("Debug Summary", debugSummary);

          // === CONSOLE OUTPUT ===
          System.out.println("=== DETAILED ROTATION DEBUG ===");
          System.out.println("Gyro Yaw (raw): " + gyroYaw + "°");
          System.out.println("Pose Rotation (degrees): " + poseRotation.getDegrees() + "°");
          System.out.println("Pose Rotation (radians): " + poseRotation.getRadians());
          System.out.println("Pose (full): " + fullPose);
          System.out.println("Difference: " + difference + "°");
          System.out.println("==============================");
        }));

    // D-Pad LEFT: Reset to Reef Tag 17 position
    xboxDriveController.povLeft().onTrue(
        new InstantCommand(() -> {
          double desiredAngle = 60.0;

          // STEP 1: Reset the Pigeon2 gyro to desired angle
          driveSubsystem.getPigeon2().setYaw(desiredAngle);

          // STEP 2: Reset the pose to same angle
          Pose2d reefPose = new Pose2d(
              3.897,
              2.957,
              Rotation2d.fromDegrees(desiredAngle));

          driveSubsystem.resetPose(reefPose);
          questNavSubsystem.resetQuestOdometry(reefPose);

          // ✅ STEP 3: Update operator perspective to match robot orientation
          driveSubsystem.setOperatorPerspectiveForward(Rotation2d.fromDegrees(desiredAngle));

          // === SMARTDASHBOARD OUTPUT ===
          SmartDashboard.putString("Reset Location", "Reef Tag 17 (60°)");
          SmartDashboard.putNumber("Reset Target Angle", desiredAngle);
          SmartDashboard.putNumber("Reset Perspective", desiredAngle + 180); // ✅ Added

          // Immediately read back and display
          double actualGyro = driveSubsystem.getPigeon2().getYaw().getValueAsDouble();
          double actualPose = driveSubsystem.getPose().getRotation().getDegrees();

          SmartDashboard.putNumber("Reset Actual Gyro", actualGyro);
          SmartDashboard.putNumber("Reset Actual Pose", actualPose);

          String resetSummary = String.format(
              "Set: %.1f° | Gyro: %.2f° | Pose: %.2f° | Perspective: %.1f°",
              desiredAngle,
              actualGyro,
              actualPose,
              desiredAngle); // ✅ Added perspective to summary
          SmartDashboard.putString("Reset Summary", resetSummary);
        }));
  }

  public void setYaws() {

    xboxDriveController.leftBumper()
        .onTrue(new InstantCommand(() -> {
          // Get current position
          Pose2d currentPose = driveSubsystem.getPose();
          double currentX = currentPose.getX();
          double currentY = currentPose.getY();
          double oldRotation = currentPose.getRotation().getDegrees();

          // Downfield = toward red (0°) for both alliances
          Rotation2d downfieldRotation = Rotation2d.kZero;

          // Create new pose: SAME position, NEW rotation
          Pose2d orientedPose = new Pose2d(currentX, currentY, downfieldRotation);

          // Reset gyro
          driveSubsystem.getPigeon2().setYaw(0);

          // Update odometry
          questNavSubsystem.resetQuestOdometry(orientedPose);
          driveSubsystem.resetPose(orientedPose);

          // Update operator perspective
          driveSubsystem.setOperatorPerspectiveForward(Rotation2d.kZero);

          // Publish to SmartDashboard
          SmartDashboard.putNumber("Orient Old Rotation", oldRotation);
          SmartDashboard.putNumber("Orient New Rotation", 0);
          SmartDashboard.putString("Orient Summary",
              String.format("(%.2f, %.2f): %.0f° → 0°", currentX, currentY, oldRotation));

          System.out.println("=== ORIENT TO DOWNFIELD ===");
          System.out.println("Position: (" + currentX + ", " + currentY + ") - UNCHANGED");
          System.out.println("Rotation: " + oldRotation + "° → 0°");
          System.out.println("Operator perspective: 0° (forward = toward red)");
          System.out.println("==========================");
        }));
  }

  public static Command runTrajectoryPathPlannerWithForceResetOfStartingPose(String tr,
      boolean shouldResetOdometryToStartingPose, boolean flipTrajectory) {

    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromPathFile(tr);

      Pose2d startPose = path.getStartingHolonomicPose().get(); // reset odometry, as PP may not do so

      // Create a path following command using AutoBuilder. This will also trigger
      // event markers.
      if (!shouldResetOdometryToStartingPose) {

        // alex test
        // System.out.println("Rigth before driving without reset");
        return AutoBuilder.followPath(path);

      } else { // reset odometry the right way

        // alex test
        // System.out.println("Rigth before driving with reset");

        return Commands.sequence(
            new InstantCommand(
                () -> questNavSubsystem.resetQuestOdometry(TrajectoryHelper.flipQuestPoseRed(startPose))),
            AutoBuilder.resetOdom(startPose), new WaitCommand(0), AutoBuilder.followPath(path));

        // return Commands.sequence(AutoBuilder.resetOdom(startPose));

        // return Commands.sequence(new InstantCommand(() ->
        // questNavSubsystem.resetQuestOdometry(TrajectoryHelper.flipQuestPoseRed(startPose))),
        // AutoBuilder.resetOdom(startPose));
      }
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  // Alliance color determination
  public void checkAllianceColor() {
    SmartDashboard.putString("AllianceColor", DriverStation.getAlliance().toString());
  }

  public static void setIfAllianceRed() {
    var alliance = DriverStation.getAlliance();
    if (!alliance.isPresent()) {
      System.out.println("=== !!! Alliance not present !!! === Staying with the BLUE system");
    } else {
      isAllianceRed = alliance.get() == DriverStation.Alliance.Red;
      System.out.println("*** RED Alliance: " + isAllianceRed);
    }
  }

  public static void toggleReversingControllerAndIMUForRed() {
    isReversingControllerAndIMUForRed = !isReversingControllerAndIMUForRed;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}