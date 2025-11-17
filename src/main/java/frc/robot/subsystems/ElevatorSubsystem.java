package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  SparkMax masterMotor;
  RelativeEncoder elEncoder;
  AlternateEncoderConfig alternateEncoderConfig;

  public ElevatorSubsystem() {
    masterMotor = new SparkMax(Constants.Elevator.masterID, MotorType.kBrushless);
    elEncoder = masterMotor.getAlternateEncoder();
    alternateEncoderConfig = new AlternateEncoderConfig();
    
  }

  public SparkMaxConfig SparkMaxConfig() {
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(IdleMode.kBrake).voltageCompensation(12).smartCurrentLimit(40);
    return motorConfig;
  }

  public void setPosition() {

  }

  public void stop() {
    masterMotor.set(0);
  }

  public double getPosition() {
    return elEncoder.getPosition();
  }

  public void holdElevatorAtPosition() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
