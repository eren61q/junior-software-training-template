package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//@Logged(name = "Shooter")
public class ShooterSubsystem extends SubsystemBase {

  private double currentVoltage;
  /** Creates a new ExampleSubsystem. */

  SparkMax masterMotor;


  public ShooterSubsystem() {
    masterMotor = new SparkMax(Constants.Shooter.motorID, MotorType.kBrushless);
  }

  public void setVoltage(double voltage) {
    masterMotor.setVoltage(voltage);
  }

  public void stop() {
    masterMotor.setVoltage(0);
  }



  public SparkMaxConfig SparkMaxConfig() {
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(IdleMode.kBrake).voltageCompensation(12).smartCurrentLimit(40);
    return motorConfig;
  }

  public Command shootCommand(double voltage) {
    return runOnce(() -> setVoltage(voltage));
  }

  public Command shooterStop() {
    return runOnce(() -> stop());
  }

  // default shoot
  public Command l1shoot() {
    return startEnd(
        () -> setVoltage(Constants.Shooter.scoreL1Voltage),
        () -> stop()).until(() -> hasCoral());
}
public Command l2l3shoot() {
  return startEnd(
      () -> setVoltage(Constants.Shooter.scoreL2L3Voltage), 
      () -> stop()).until(() -> hasCoral());
}
public Command l4shoot() {
  return startEnd(
      () -> setVoltage(Constants.Shooter.scoreL4Voltage), 
      () -> stop()).until(() -> hasCoral());
}


  public boolean  hasCoral(){
    boolean hasCoral = SmartDashboard.getBoolean("hasCoral", false);
    return hasCoral;
  }

  @Override
  public void periodic() {
    currentVoltage = masterMotor.getAppliedOutput() * 12;
    SmartDashboard.putNumber("Current Voltage", currentVoltage);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    //no
  }

}