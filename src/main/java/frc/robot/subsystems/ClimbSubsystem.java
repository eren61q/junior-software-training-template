package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//@Logged(name = "Climb")
public class ClimbSubsystem extends SubsystemBase {

    private final SparkMax masterMotor;

    private ClimbState state = ClimbState.IDLE;

    public void setState(ClimbState cState){
        state = cState;
    }

    public Command openClimb() {
        return runOnce(() -> setState(ClimbState.OPENING));
    }

    public Command holdClimbCommand() {
        return runOnce(() -> setState(ClimbState.HOLDING));
    }

    public Command closeClimb() {
        return runOnce(() -> setState(ClimbState.CLOSING));
    }

    public ClimbSubsystem() {
        masterMotor = new SparkMax(Constants.Climb.Climber.motorID, MotorType.kBrushless);
    }

    public enum ClimbState {
        IDLE,
        OPENING,
        CLOSING,
        HOLDING
    }

    public void fastDeploy() {
        masterMotor.setVoltage(Constants.Climb.Climber.fastDeployVoltage);
    }

    public void slowDeploy() {
        masterMotor.setVoltage(Constants.Climb.Climber.slowDeployVoltage);
    }

    public void hold() {
        masterMotor.setVoltage(Constants.Climb.Climber.holdVoltage);
    }

    public void fastRetract() {
        masterMotor.setVoltage(Constants.Climb.Climber.fastRetractVoltage);
    }

    public void slowRetract() {
        masterMotor.setVoltage(Constants.Climb.Climber.slowRetractVoltage);
    }

    public SparkMaxConfig SparkMaxConfig() {
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.idleMode(IdleMode.kBrake).voltageCompensation(12).smartCurrentLimit(40);
        return motorConfig;
    }

    public void periodic() {
        switch (state) {
            case OPENING:

                fastDeploy();
                
                break;

            case CLOSING:

                fastRetract();
                break;

            case HOLDING:

                hold();
                break;

            case IDLE:
            default:
                masterMotor.setVoltage(0);
                break;
        }
        SmartDashboard.putString("State: ", state.toString());
        SmartDashboard.putNumber("Voltage: ", masterMotor.getAppliedOutput());
    }
}
