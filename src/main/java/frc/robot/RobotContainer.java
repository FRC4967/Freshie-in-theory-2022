package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ComplexAuto;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    // A chooser for autonomous commands
    private SendableChooser<Command> m_chooser = new SendableChooser<>();

    private final Command m_simpleAuto = new DriveDistance(AutoConstants.kAutoDriveDistanceInches,
            AutoConstants.kAutoDriveSpeed, m_robotDrive);

    // A complex auto routine that drives forward, drops a hatch, and then drives
    // backward.
    private final Command m_complexAuto = new ComplexAuto(m_robotDrive);

    // The driver's controller
    XboxController m_driverController = new XboxController(0);

    public RobotContainer() {

        m_robotDrive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new DefaultDrive(
                        m_robotDrive, m_driverController::getLeftY, m_driverController::getRightX));

        m_chooser.setDefaultOption("Simple Auto", m_simpleAuto);
        m_chooser.addOption("Complex Auto", m_complexAuto);

        // Put the chooser on the dashboard
        Shuffleboard.getTab("Autonomous").add(m_chooser);
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

}
