package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDrive extends CommandBase {

    private final DriveSubsystem drive;
    private final DoubleSupplier forward;
    private final DoubleSupplier rotation;

    /**
     * Creates a new DefaultDrive.
     *
     * @param subsystem The drive subsystem this command wil run on.
     * @param forward   The control input for driving forwards/backwards
     * @param rotation  The control input for turning
     */
    public DefaultDrive(DriveSubsystem subsystem, DoubleSupplier forward, DoubleSupplier rotation) {
        this.drive = subsystem;
        this.forward = forward;
        this.rotation = rotation;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.arcadeDrive(-forward.getAsDouble(), rotation.getAsDouble());
    }

}
