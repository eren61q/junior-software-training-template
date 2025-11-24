package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends SequentialCommandGroup{

    private final IntakeSubsystem intake;

    public IntakeCommand(IntakeSubsystem m_intake){
        intake = m_intake;
        addRequirements(m_intake);
        if (!intake.hasCoral()){
            addCommands(
            new InstantCommand(() -> intake.intakeIn()),
            new WaitCommand(0.4));
        }
        else{
            addCommands(
            new InstantCommand(() -> intake.intakeOut()),
            new WaitCommand(0.4));
        }
    }


    
}
