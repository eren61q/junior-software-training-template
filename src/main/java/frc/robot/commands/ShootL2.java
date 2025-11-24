package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootL2 extends SequentialCommandGroup{

    private final ElevatorSubsystem elevator;
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;

    public ShootL2(ElevatorSubsystem m_elevator, ShooterSubsystem m_shooter, IntakeSubsystem m_intake){
        elevator = m_elevator;
        shooter = m_shooter;
        intake = m_intake;
        addRequirements(m_elevator, m_shooter);

        if(intake.hasCoral()){addCommands(
            new InstantCommand(() -> elevator.l2Score()),
            new WaitCommand(0.4));
            new InstantCommand(() -> shooter.l2l3shoot());}
    }


    
}
