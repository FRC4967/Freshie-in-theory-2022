package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistanceProfiled extends TrapezoidProfileCommand {

    // private final DriveSubsystem m_drive;
    // private final double m_distance;
    // private final double m_speed;

    /**
     * Creates a new DriveDistance.
     *
     * @param meters The number of inches the robot will drive
     * @param speed  The speed at which the robot will drive
     * @param drive  The drive subsystem on which this command will run
     */
    public DriveDistanceProfiled(double meters, DriveSubsystem drive) {
        super(
                new TrapezoidProfile(
                        // Limit the max acceleration and velocity
                        new TrapezoidProfile.Constraints(
                                DriveConstants.kMaxSpeedMetersPerSecond,
                                DriveConstants.kMaxAccelerationMetersPerSecondSquared),
                        // End at desired position in meters; implicitly starts at 0
                        new TrapezoidProfile.State(meters, 0)),
                // Pipe the profile state to the drive
                setpointState -> drive.setDriveStates(setpointState, setpointState),
                // Require the drive
                drive);
        // Reset drive encoders since we're starting at 0
        drive.resetEncoders();
    }

    // @Override
    // public void initialize() {
    //     m_drive.resetEncoders();
    //     m_drive.arcadeDrive(m_speed, 0);
    // }

    // @Override
    // public void execute() {
    //     m_drive.arcadeDrive(m_speed, 0);

    // }

    // @Override
    // public void end(boolean interrupted) {
    //     m_drive.arcadeDrive(0, 0);
    // }

    // @Override
    // public boolean isFinished() {
    //     double currentDistance = m_drive.getAverageEncoderDistance();
    //     System.out.println(String.format("%.4f --> %.4f", currentDistance, m_distance));
    //     return Math.abs(currentDistance) >= m_distance;
    // }
}
