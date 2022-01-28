package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistance extends CommandBase {

    private final DriveSubsystem drive;
    private final double distance;
    private final double speed;

    /**
     * Creates a new DriveDistance.
     *
     * @param inches The number of inches the robot will drive
     * @param speed  The speed at which the robot will drive
     * @param drive  The drive subsystem on which this command will run
     */
    public DriveDistance(double inches, double speed, DriveSubsystem drive) {
        this.distance = inches;
        this.speed = speed;
        this.drive = drive;
        addRequirements(drive);
    }
}
