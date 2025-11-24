package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootL3 extends SequentialCommandGroup{

    private final ElevatorSubsystem elevator;
    private final ShooterSubsystem shooter;

    public ShootL3(ElevatorSubsystem m_elevator, ShooterSubsystem m_shooter){
        elevator = m_elevator;
        shooter = m_shooter;
        addRequirements(m_elevator, m_shooter);

        addCommands(
            new InstantCommand(() -> elevator.l3Score()),
            new WaitCommand(0.4));
            new InstantCommand(() -> shooter.l2l3shoot());
    }


    
}
