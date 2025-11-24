package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ClimbSubsystem;

public class CloseClimb extends SequentialCommandGroup {

    private final ClimbSubsystem climb;

    public CloseClimb(ClimbSubsystem m_climb) {
        climb = m_climb;
        addRequirements(m_climb);
        addCommands(
                new InstantCommand(() -> climb.closeClimb()),
                new WaitCommand(0.4));

    }
}