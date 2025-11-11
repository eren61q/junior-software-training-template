public class IntakeSubsystem {
    

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

Logged(@name = "Intake")

SparkMax motor = new SparkMax(canID, MotorType.kBrushless);

public class IntakeSubsystem{
    SparkMax intakeMotor = new SparkMax(Constants.Intake.intakeMotor, MotorType.kBrushless);
    
    public double getVoltage() {
        return intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
}
  
    public void setVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);

    public void stop()
        intakeMotor.setVoltage(0);

    public boolean hasCoral(){
        return sensor.get();
    }

}}