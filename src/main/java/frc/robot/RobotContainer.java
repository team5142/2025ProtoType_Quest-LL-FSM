// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.OperatorConstants.OIContants;
import frc.robot.Constants.OperatorConstants.SwerveConstants;
import frc.robot.Constants.OperatorConstants.OIContants.ControllerDevice;
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

  // Slew rate limiters for smooth acceleration
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(3); // 3 m/s² acceleration
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(3); // 3 m/s² acceleration
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(1.5); // 6 rad/s² rotational acceleration

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
    testTrajectory();
    setYaws();

    driveSubsystem.setDefaultCommand(
        new DriveManuallyCommand(
            () -> getDriverLeftRight(),
            () -> getDriverForwardBackward(),
            () -> getDriverOmegaAxis()));
    FollowPathCommand.warmupCommand().schedule();
  }

  // Driver controls - properly scaled to max speeds
  private double getDriverForwardBackward() {
    double rawValue = xboxDriveController.getLeftY();

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
    double rawValue = xboxDriveController.getLeftX();

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
    xboxDriveController.back().onTrue(
        new InstantCommand(() -> {
          System.out.println("=== Limelight Test ===");
          String cam = LLCamera.LLBACK.getCameraName();
          System.out.println("Testing camera: " + cam);

          try {
            boolean hasTarget = LimelightHelpers.getTV(cam);
            double tagID = LimelightHelpers.getFiducialID(cam);
            // Null check before accessing pose
            var pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(cam);

            System.out.println("Has Target: " + hasTarget);
            System.out.println("Tag ID: " + (int) tagID);

            if (pose != null) {
              System.out.println("Tag Count: " + pose.tagCount);
              System.out.println("Pose: " + pose.pose);
              System.out.println("Latency: " + pose.latency + "ms");

              if (RobotContainer.llAprilTagSubsystem.isAnyReefTagID((int) tagID)) {
                System.out.println("✅ REEF TAG!");
              }
            } else {
              System.out.println("❌ Pose is NULL - camera not connected or wrong name");
            }

          } catch (Exception e) {
            System.out.println("❌ ERROR: " + e.getMessage());
            e.printStackTrace();
          }
        }));
  }

  public Command stopRobotCommand() {
    System.out.println("***Stopping Robot");
    return driveSubsystem.applyRequest(() -> driveSubsystem.getDrive().withVelocityX(0) // Drive forward with negative Y
                                                                                        // (forward)
        .withVelocityY(0) // Drive left with negative X (left)
        .withRotationalRate(0) // Drive counterclockwise with negative X (left)

    );
  }

  private void testTrajectory() {
    xboxDriveController.povUp().onTrue(new TwoMeterTest());
    // D-Pad RIGHT: L-shape test
    xboxDriveController.povRight().onTrue(new LShapeTest());

    xboxDriveController.povDown().onTrue(
        new InstantCommand(() -> {
          System.out.println("=== HEADING DEBUG ===");
          System.out.println("Robot Heading: " + driveSubsystem.getPose().getRotation().getDegrees() + "°");
          System.out.println("IMU Yaw: " + driveSubsystem.getPigeon2().getYaw().getValueAsDouble() + "°");
          System.out.println("Pose: " + driveSubsystem.getPose());
          System.out.println("==================");
        }));

    xboxDriveController.a()
        .onTrue(new OneMeterForwardPPCommand());
    // new JoystickButton(xboxDriveController, 2)
    // .onTrue(new ThreeMeterForwardPPCommand());
    xboxDriveController.b()
        .onTrue(new ReturnTestPPCommand())
        .onFalse(new StopRobot());
    xboxDriveController.x()
        .onTrue(new InstantCommand(() -> questNavSubsystem.resetQuestOdometry(new Pose2d(10, 10, Rotation2d.k180deg))));
    xboxDriveController.y()
        .whileTrue(new PathPlannerAuto("Reef Off"))
        .onFalse(new StopRobot());
    xboxDriveController.rightBumper()
        .whileTrue(new PathPlannerAuto("Reef off and Reef on"))
        .onFalse(new StopRobot());
  }

  public void setYaws() {
    // Button 8 (Start): Zero chassis and Quest yaw
    /*
     * xboxDriveController.start()
     * .onTrue(new InstantCommand(() -> driveSubsystem.zeroChassisYaw())
     * .andThen(new InstantCommand(() -> questNavSubsystem.zeroYaw())));
     * 
     */
    // Button 7 (Back): Reset Quest to zero pose
    xboxDriveController.back()
        .onTrue(new InstantCommand(() -> questNavSubsystem.resetToZeroPose()));
    /*
     * 
     * xboxDriveController.leftTrigger(0.5).whileTrue(
     * Commands.startEnd(
     * () -> odometryUpdateSubsystem.enableHeadingUpdatesForCommand(),
     * () -> odometryUpdateSubsystem.disableHeadingUpdatesForCommand()
     * )
     * );
     * 
     */
    // Button 5 (Left Bumper): Reset to downfield starting pose

    xboxDriveController.leftBumper()
        .onTrue(new InstantCommand(() -> {
          // Determine pose AND heading based on alliance color
          Pose2d downfieldPose;

          if (isAllianceRed) {
            // Red alliance - downfield facing toward Blue (0°)
            downfieldPose = new Pose2d(15.0, 5.5, Rotation2d.kZero);
          } else {
            // Blue alliance - downfield facing toward Red (180°)
            downfieldPose = new Pose2d(1.5, 5.5, Rotation2d.k180deg);
          }

          // Reset both Quest and drive odometry
          questNavSubsystem.resetQuestOdometry(downfieldPose);
          driveSubsystem.resetPose(downfieldPose);

          System.out.println("*** Full reset to downfield: " + downfieldPose);
        }));
  }

  public static Command runTrajectoryPathPlannerWithForceResetOfStartingPose(String tr,
      boolean shouldResetOdometryToStartingPose, boolean flipTrajectory) {

    // alex test
    System.out.println("Start drive routine");

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

  // alex test
  // public static Command testCommand2() {
  // return new PrintCommand("Test 2 Command");
  // }

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