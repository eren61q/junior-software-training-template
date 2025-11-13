import main.java.frc.robot.Constants;
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
import main.java.frc.robot.Constants;
import main.java.frc.robot.subsystems.ElevatorSubsystem.ControlMode;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


@Logged(name = "Intake")
public class ClimbSubsystem {

    private final SparkMax masterMotor = new SparkMax(Constants.Climb.masterMotor, MotorType.kBrushless);
    private final SparkMax slaveMotor = new SparkMax(Constants.Climb.slaveMotor, MotorType.kBrushless);

    private final RelativeEncoder masterEncoder = masterMotor.getEncoder();
    private final RelativeEncoder slaveEncoder = slaveMotor.getEncoder();

    private final PIDController pidController = new PIDController();

    private double targetMasterPosition;
    private double targetSlavePosition;

    private double ffMaster = 1; //determine
    private double ffSlave = 1; //determine

    private ClimbState state = ClimbState.IDLE;

    public enum ClimbState {
        IDLE,
        OPENING,
        CLOSING,
        HOLDING
    }

    public double getMasterAngle() {
        return masterEncoder.getPosition() * 360.0; //position in angle
    }

    public double getSlaveAngle() {
        return slaveEncoder.getPosition() * 360.0;  //position in angle
    }

    public void setState(ClimbState newState) {
        state = newState;

        switch (state) {
            case HOLDING:
                targetMasterPosition = masterEncoder.getPosition();
                targetSlavePosition = slaveEncoder.getPosition();
                break;

            case OPENING:
                targetMasterPosition += 1.0; // adjust
                targetSlavePosition += 1.0;
                break;

            case CLOSING:
                targetMasterPosition -= 1.0;
                targetSlavePosition -= 1.0;
                break;

            case IDLE:
            default:
                // do nothing
                break;
        }
  }

    //should be in periodic(check later)
    public void update() {
        switch (state) {
            case OPENING:
                double masterVoltage = pidController.calculate(getMasterAngle(), targetMasterPosition * 360.0);
                double slaveVoltage = pidController.calculate(getSlaveAngle(), targetSlavePosition * 360.0);

                masterMotor.setVoltage(masterVoltage);
                slaveMotor.setVoltage(slaveVoltage);
                break;
            case CLOSING:
                double masterVoltage = pidController.calculate(getMasterAngle(), targetMasterPosition * 360.0);
                double slaveVoltage = pidController.calculate(getSlaveAngle(), targetSlavePosition * 360.0);

                masterMotor.setVoltage(masterVoltage);
                slaveMotor.setVoltage(slaveVoltage);
                break;
            case HOLDING:

                masterMotor.setVoltage(ffMaster);
                slaveMotor.setVoltage(ffSlave);
                break;

            case IDLE:
            default:
                masterMotor.setVoltage(0);
                slaveMotor.setVoltage(0);
                break;
    }
}   
}