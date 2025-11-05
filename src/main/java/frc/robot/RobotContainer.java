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

    /* private static final double TELEOP_SPEED_MULTIPLIER = 0.5; // 50% speed
    // Rate limiters to prevent brownouts from instant joystick inputs
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(2.0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(2.0);
    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(3.0); */

    public RobotContainer() {
        configureBindings();
        testTrajectory();
        setYaws();

        driveSubsystem.setDefaultCommand(
      new DriveManuallyCommand(
          () -> getDriverYAxis(),
          () -> getDriverXAxis(),
          () -> getDriverOmegaAxis()));
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // driveSubsystem.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     driveSubsystem.applyRequest(() ->
        //         driveSubsystem.getDrive().withVelocityX(-xboxDriveController.getLeftY() * SwerveConstants.MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-xboxDriveController.getLeftX() * SwerveConstants.MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-xboxDriveController.getRightX() * SwerveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        RobotModeTriggers.disabled().whileTrue(driveSubsystem.applyRequest(() -> driveSubsystem.getIdle()).ignoringDisable(true));

        // xboxDriveController.a().whileTrue(driveSubsystem.applyRequest(() -> driveSubsystem.getBrake()));
        // xboxDriveController.b().whileTrue(driveSubsystem.applyRequest(() ->
        //     driveSubsystem.getPoint().withModuleDirection(new Rotation2d(-xboxDriveController.getLeftY(), -xboxDriveController.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // xboxDriveController.back().and(xboxDriveController.y()).whileTrue(driveSubsystem.sysIdDynamic(Direction.kForward));
        // xboxDriveController.back().and(xboxDriveController.x()).whileTrue(driveSubsystem.sysIdDynamic(Direction.kReverse));
        // xboxDriveController.start().and(xboxDriveController.y()).whileTrue(driveSubsystem.sysIdQuasistatic(Direction.kForward));
        // xboxDriveController.start().and(xboxDriveController.x()).whileTrue(driveSubsystem.sysIdQuasistatic(Direction.kReverse));

        // // reset the field-centric heading on left bumper press
        // xboxDriveController.leftBumper().onTrue(driveSubsystem.runOnce(() -> driveSubsystem.seedFieldCentric()));

        driveSubsystem.registerTelemetry(logger::telemeterize);

        // xboxDriveController.x().onTrue(new QuestNavTrajectoryTest())
        //             .onFalse(stopRobotCommand());
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
            System.out.println("Tag ID: " + (int)tagID);
            
            if (pose != null) {
                System.out.println("Tag Count: " + pose.tagCount);
                System.out.println("Pose: " + pose.pose);
                System.out.println("Latency: " + pose.latency + "ms");
                
                if (RobotContainer.llAprilTagSubsystem.isAnyReefTagID((int)tagID)) {
                    System.out.println("✅ REEF TAG!");
                }
            } else {
                System.out.println("❌ Pose is NULL - camera not connected or wrong name");
            }
            
        } catch (Exception e) {
            System.out.println("❌ ERROR: " + e.getMessage());
            e.printStackTrace();
        }
    })
    );
    }

    public Command stopRobotCommand() {
        System.out.println("***Stopping Robot");
        return driveSubsystem.applyRequest(() ->
                driveSubsystem.getDrive().withVelocityX(0) // Drive forward with negative Y (forward)
                    .withVelocityY(0) // Drive left with negative X (left)
                    .withRotationalRate(0) // Drive counterclockwise with negative X (left)
                    
        );
    }

    private void testTrajectory() {
      xboxDriveController.povUp().onTrue(new TwoMeterTest());
      // D-Pad RIGHT: L-shape test
      xboxDriveController.povRight().onTrue(new LShapeTest());
      
      xboxDriveController.a()
        .onTrue(new OneMeterForwardPPCommand());
        // new JoystickButton(xboxDriveController, 2)
        // .onTrue(new ThreeMeterForwardPPCommand());
      xboxDriveController.b()
        .onTrue(new ReturnTestPPCommand())
        .onFalse(new StopRobot());
      xboxDriveController.x()
        .onTrue(new InstantCommand(() -> questNavSubsystem.resetQuestOdometry(new Pose2d(10, 10, Rotation2d.k180deg))));

      /* new JoystickButton(xboxDriveController, 4)
        .onTrue(questNavSubsystem.offsetTranslationCharacterizationCommand())
        .onFalse(new StopRobot());

      new JoystickButton(xboxDriveController, 6)
        .onTrue(questNavSubsystem.offsetAngleCharacterizationCommand())
        .onFalse(new StopRobot()); */
      xboxDriveController.y()
        .whileTrue(new PathPlannerAuto("Reef Off"))
        .onFalse(new StopRobot());
        // Option 1: If getRightTrigger() returns > 0.5 automatically as boolean
      /* new JoystickButton(xboxDriveController, 5)
        .whileTrue(new PathPlannerAuto("Reef Bounce"))
        .onFalse(new StopRobot()); */
      xboxDriveController.rightBumper()
        .whileTrue(new PathPlannerAuto("Reef off and Reef on"))
        .onFalse(new StopRobot());
    }
    
    /*
     * // Button 3: Reef Bounce auto routine
        joystick.button(3).whileTrue(
          Commands.deferredProxy(() -> autoChooser.getSelected())
        ).onFalse(new StopRobot(driveSubsystem));
      
        // Button 4: Reef Off auto routine (NEW - replaces QuestNav characterization)
        joystick.button(4).whileTrue(
          new PathPlannerAuto("Reef Off")
        ).onFalse(new StopRobot(driveSubsystem));
        
        // QuestNav characterization (COMMENTED OUT - not needed frequently)
        // joystick.button(4).whileTrue(new QuestOffsetCharacterization(questNavSubsystem, driveSubsystem));
      
        // Button 6: Reef Off, Reef On auto routine (NEW - replaces test command)
        joystick.button(6).whileTrue(
          new PathPlannerAuto("Reef Off, Reef On")
        ).onFalse(new StopRobot(driveSubsystem));
        
        // QuestNav trajectory test (COMMENTED OUT - not needed frequently)
        // joystick.button(6).whileTrue(new QuestNavTrajectoryTest(questNavSubsystem, driveSubsystem));
     * 
     * 
     */

    public void setYaws() {
      // Button 8 (Start): Zero chassis and Quest yaw
      xboxDriveController.start()
          .onTrue(new InstantCommand(() -> driveSubsystem.zeroChassisYaw())
              .andThen(new InstantCommand(() -> questNavSubsystem.zeroYaw())));
      
      // Button 7 (Back): Reset Quest to zero pose
      xboxDriveController.back()
          .onTrue(new InstantCommand(() -> questNavSubsystem.resetToZeroPose()));
      
      // Button 5 (Left Bumper): Reset to downfield starting pose
      xboxDriveController.leftBumper()
      .onTrue(new InstantCommand(() -> {
          // Get current robot heading to preserve orientation
          Rotation2d currentHeading = driveSubsystem.getState().Pose.getRotation();
          
          // Determine pose based on alliance color
          Pose2d downfieldPose;
          if (isAllianceRed) {
              // Red alliance - downfield on opposite side, keeps current heading
              downfieldPose = new Pose2d(15.0, 5.5, currentHeading);
          } else {
              // Blue alliance - downfield, keeps current heading
              downfieldPose = new Pose2d(1.5, 5.5, currentHeading);
          }
          
          // Reset both Quest and drive odometry
          questNavSubsystem.resetQuestOdometry(downfieldPose);
          driveSubsystem.resetPose(downfieldPose);
          
          System.out.println("*** Reset to downfield pose with heading: " + currentHeading.getDegrees() + "°");
      }));
    }

    
     // Driver controls - properly scaled to max speeds
     private double getDriverXAxis() {
      // Left stick Y controls forward/backward (field X-axis)
      // Inverted because joystick up = -1.0, but we want positive = forward
      return xboxDriveController.getLeftX() * SwerveConstants.MaxSpeed;
    }
  
    private double getDriverYAxis() {
      // Left stick X controls left/right strafe (field Y-axis)
      // Inverted because joystick right = +1.0, but field Y-axis points left
      return xboxDriveController.getLeftY() * SwerveConstants.MaxSpeed;
    }
  
    private double getDriverOmegaAxis() {
      // Right stick X controls rotation
      // Inverted for intuitive controls (right stick right = clockwise)
      return -xboxDriveController.getRightX() * SwerveConstants.MaxAngularRate;
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
      if (! shouldResetOdometryToStartingPose) {

        // alex test
        //System.out.println("Rigth before driving without reset");
        return AutoBuilder.followPath(path);

      } else { // reset odometry the right way

        // alex test
        //System.out.println("Rigth before driving with reset");

        return Commands.sequence(new InstantCommand(() -> 
          questNavSubsystem.resetQuestOdometry(TrajectoryHelper.flipQuestPoseRed(startPose))), 
            AutoBuilder.resetOdom(startPose), new WaitCommand(0), AutoBuilder.followPath(path));
        
          //return Commands.sequence(AutoBuilder.resetOdom(startPose));

        // return Commands.sequence(new InstantCommand(() -> 
        //   questNavSubsystem.resetQuestOdometry(TrajectoryHelper.flipQuestPoseRed(startPose))), AutoBuilder.resetOdom(startPose));
      }
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  // alex test
  // public static Command testCommand2() {
  //   return new PrintCommand("Test 2 Command");
  // }

    // Alliance color determination
    public void checkAllianceColor() {
      SmartDashboard.putString("AllianceColor", DriverStation.getAlliance().toString());
    }
  
    public static void setIfAllianceRed() {
      var alliance = DriverStation.getAlliance();
      if (! alliance.isPresent()) {
          System.out.println("=== !!! Alliance not present !!! === Staying with the BLUE system");
      } else {
          isAllianceRed = alliance.get() == DriverStation.Alliance.Red;
          System.out.println("*** RED Alliance: "+isAllianceRed);
      }
    }
    public static void toggleReversingControllerAndIMUForRed() {
      isReversingControllerAndIMUForRed = !isReversingControllerAndIMUForRed;
    }



    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}