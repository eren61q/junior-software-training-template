package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  SparkMax masterMotor;
  RelativeEncoder elEncoder;
  AlternateEncoderConfig alternateEncoderConfig;
  PIDController pidController;
  private double targetPosition;
  ElevatorFeedforward ff = new ElevatorFeedforward(Constants.Elevator.kS, Constants.Elevator.kG, Constants.Elevator.kV);
  double voltage = ff.calculate(masterMotor.get());

  public ElevatorSubsystem() {
    masterMotor = new SparkMax(Constants.Elevator.masterID, MotorType.kBrushless);
    elEncoder = masterMotor.getAlternateEncoder();
    alternateEncoderConfig = new AlternateEncoderConfig();
    pidController = new PIDController(2.2, 0, 0);

  }

  public SparkMaxConfig SparkMaxConfig() {
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(IdleMode.kBrake).voltageCompensation(12).smartCurrentLimit(40);
    return motorConfig;
  }

  public void setVoltage(double voltage) {
    masterMotor.setVoltage(voltage);
  }

  public void setPosition(double newTargetMeters) {
    targetPosition = newTargetMeters;
    masterMotor.setVoltage(pidController.calculate(getPosition(), targetPosition));
  }

  public void stop() {
    masterMotor.set(0);
  }

  public double getPosition() {
    return elEncoder.getPosition();
  }

  public void holdElevatorAtPosition() {
    masterMotor.setVoltage(voltage);

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

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
