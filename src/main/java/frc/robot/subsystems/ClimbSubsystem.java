package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import java.util.function.DoubleSupplier;

@Logged(name = "Intake")
public class ClimbSubsystem {

    private String stateName;

    private final SparkMax masterMotor;

    private final RelativeEncoder masterEncoder;

    private double targetMasterPosition;

    private double ffMaster = 1; // determine

    private ClimbState state = ClimbState.IDLE;

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
        masterEncoder = masterMotor.getEncoder();
    }

    public enum ClimbState {
        IDLE,
        OPENING,
        CLOSING,
        HOLDING
    }

    public double getMasterAngle() {
        return masterEncoder.getPosition() * 360.0; // position in angle
    }

    public void setState(ClimbState newState) {
        state = newState;

        switch (state) {
            case HOLDING:
                targetMasterPosition = masterEncoder.getPosition();

                break;

            case OPENING:
                targetMasterPosition += 1.0; // adjust

                break;

            case CLOSING:
                targetMasterPosition -= 1.0;

                break;

            case IDLE:
            default:
                // do nothing
                break;
        }
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

                stateName = "Opening";
                fastDeploy();
                break;

            case CLOSING:

                stateName = "Closing";
                fastRetract();
                break;

            case HOLDING:

                stateName = "Holding";
                hold();
                break;

            case IDLE:
            default:
                masterMotor.setVoltage(0);
                break;
        }
        SmartDashboard.putString("State: ", stateName);
        SmartDashboard.putNumber("Voltage: ", masterMotor.getAppliedOutput());
    }
}
