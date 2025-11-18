package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//@Logged(name = "Shooter")
public class ShooterSubsystem extends SubsystemBase {

  private double currentVoltage;
  /** Creates a new ExampleSubsystem. */

  SparkMax masterMotor;
  SparkMaxConfig motorConfig = new SparkMaxConfig();
  

  public ShooterSubsystem() {
    masterMotor = new SparkMax(Constants.Shooter.motorID, MotorType.kBrushless);
    motorConfig.idleMode(IdleMode.kBrake).voltageCompensation(12).smartCurrentLimit(40);
    masterMotor.configure(motorConfig, null, null);
  }

  public void setVoltage(double voltage) {
    masterMotor.setVoltage(voltage);
  }

  public void stop() {
    masterMotor.setVoltage(0);
  }



 

  public Command shootCommand(double voltage) {
    return runOnce(() -> setVoltage(voltage));
  }

  public Command shooterStop() {
    return runOnce(() -> stop());
  }

  // default shoot
  public Command timedShoot(double voltage, double seconds) {
    return startEnd(
        () -> setVoltage(voltage), // başlarken ve periodic çalışırken voltaj uygula
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