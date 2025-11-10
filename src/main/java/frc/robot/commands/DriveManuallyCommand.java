package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants.SwerveConstants;

/**
 * Command for manual teleop driving using field-centric control
 */
public class DriveManuallyCommand extends Command {

    private DoubleSupplier mVxSupplier;
    private DoubleSupplier mVySupplier;
    private DoubleSupplier mOmegaSupplier;

    /**
     * Create a manual drive command
     * 
     * @param vxSupplier    Forward/backward velocity supplier (-1 to 1)
     * @param vySupplier    Left/right velocity supplier (-1 to 1)
     * @param omegaSupplier Rotational velocity supplier (-1 to 1)
     */
    public DriveManuallyCommand(
            DoubleSupplier vxSupplier,
            DoubleSupplier vySupplier,
            DoubleSupplier omegaSupplier) {
        this.mVxSupplier = vxSupplier;
        this.mVySupplier = vySupplier;
        this.mOmegaSupplier = omegaSupplier;

        addRequirements(RobotContainer.driveSubsystem);
    }

    @Override
    public void initialize() {
        // Nothing to initialize
    }

    @Override
    public void execute() {
        // Get joystick inputs (with 80% speed limit for safety)
        double xInput = mVxSupplier.getAsDouble() * 0.8;
        double yInput = mVySupplier.getAsDouble() * 0.8;
        double omegaInput = mOmegaSupplier.getAsDouble();

        // Always use field-centric drive
        // Operator perspective handles alliance-relative control
        RobotContainer.driveSubsystem.drive(
                xInput * SwerveConstants.MaxSpeed,
                yInput * SwerveConstants.MaxSpeed,
                omegaInput * SwerveConstants.MaxAngularRate);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when command ends
        RobotContainer.driveSubsystem.drive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return false; // Command runs until interrupted
    }
}