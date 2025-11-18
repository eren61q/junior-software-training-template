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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;


public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private MechanismLigament2d m_elevator;
  private SparkMax masterMotor;
  private RelativeEncoder elEncoder;
  private AlternateEncoderConfig alternateEncoderConfig;
  private PIDController pidController;
  private double targetPosition;
  ElevatorFeedforward ff = new ElevatorFeedforward(Constants.Elevator.kS, Constants.Elevator.kG, Constants.Elevator.kV);

  public ElevatorSubsystem() {
    masterMotor = new SparkMax(Constants.Elevator.masterID, MotorType.kBrushless);
    elEncoder = masterMotor.getAlternateEncoder();
    alternateEncoderConfig = new AlternateEncoderConfig();
    pidController = new PIDController(Constants.Elevator.kP, Constants.Elevator.kI, Constants.Elevator.kD);
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(IdleMode.kBrake).voltageCompensation(12).smartCurrentLimit(40);
    masterMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    Mechanism2d mech = new Mechanism2d(3, 3);
    MechanismRoot2d root = mech.getRoot("climber", 2, 0);
    m_elevator = root.append(new MechanismLigament2d("elevator", Constants.Climb.Levels.baseHeight, 90));
    SmartDashboard.putData("Mech2d", mech);
  }



  public void setVoltage(double voltage) {
    masterMotor.setVoltage(voltage);
  }

  public void setPosition(double newTargetMeters) {
    targetPosition = newTargetMeters;
    masterMotor.setVoltage(pidController.calculate(getPosition(), targetPosition) + ff.calculate(masterMotor.get()));
  }

  public void stop() {
    masterMotor.set(0);
  }

  public double getPosition() {
    return elEncoder.getPosition();
  }

  public void holdElevatorAtPosition() {
    masterMotor.setVoltage(ff.calculate(masterMotor.get()));

  }

  public Command holdEl() {
    return runOnce(() -> holdElevatorAtPosition());

  }

  public Command stopEl() {
    return runOnce(() -> stop());

  }

  public Command l1Score() {
    return runOnce(() -> setPosition(Constants.Climb.Levels.L1_SCORE));
  }

  public Command l2Score() {

    return runOnce(() -> setPosition(Constants.Climb.Levels.L2));
  }

  public Command l3Score() {
    return runOnce(() -> setPosition(Constants.Climb.Levels.L3));
  }

  public Command l4Score() {
    return runOnce(() -> setPosition(Constants.Climb.Levels.L4_SCORE));
  }
  
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current Position", getPosition());
    SmartDashboard.putNumber("Current Voltage", masterMotor.getAppliedOutput());

    m_elevator.setLength(Constants.Climb.Levels.baseHeight + masterMotor.getEncoder().getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
