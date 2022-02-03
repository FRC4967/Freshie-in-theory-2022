package frc.robot;

import org.junit.Test;

import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class RobotContainerTest {
    
    @Test
    public void testCreateTrajectoryCommand() {

        var container = new RobotContainer();

        RamseteCommand command = (RamseteCommand) container.createTrajectoryCommand();


    }


}
