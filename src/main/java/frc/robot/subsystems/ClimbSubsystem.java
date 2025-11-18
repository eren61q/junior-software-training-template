package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//@Logged(name = "Intake")
public class ClimbSubsystem {

    private final SparkMax masterMotor;

    private final RelativeEncoder masterEncoder;

    private double targetMasterPosition;

    private double ffMaster = 1; // determine

    private ClimbState state = ClimbState.IDLE;

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

    // should be in periodic(check later)
    public void update() {
        switch (state) {
            case OPENING:

                masterMotor.setVoltage(12);

                break;
            case CLOSING:

                masterMotor.setVoltage(-12);

                break;
            case HOLDING:

                masterMotor.setVoltage(ffMaster);
                break;

            case IDLE:
            default:
                masterMotor.setVoltage(0);
                break;
        }

    }

    public SparkMaxConfig SparkMaxConfig() {
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.idleMode(IdleMode.kBrake).voltageCompensation(12).smartCurrentLimit(40);
        return motorConfig;
    }

}