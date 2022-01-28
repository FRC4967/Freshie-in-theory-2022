package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    // The motors on the right side of the drive.
    private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(
            new WPI_TalonSRX(DriveConstants.kRightMotor1Port),
            new WPI_VictorSPX(DriveConstants.kRightMotor2Port));  
    
    // The motors on the left side of the drive.
    private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(
            new WPI_TalonSRX(DriveConstants.kLeftMotor1Port),
            new WPI_VictorSPX(DriveConstants.kLeftMotor2Port));

    // The robot's drive
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        m_rightMotors.setInverted(true);
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd, rot);
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }
}
