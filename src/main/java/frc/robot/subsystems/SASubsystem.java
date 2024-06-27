package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class SASubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  TalonFX m_shooteronemotor = new TalonFX(41);
  TalonFX m_shootertwomotor = new TalonFX(42);
  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

  CANSparkMax m_armonemotor = new CANSparkMax(43, MotorType.kBrushless);
  CANSparkMax m_armtwomotor = new CANSparkMax(44, MotorType.kBrushless);
  double kVG;
  double desiredAngle;
  private SparkPIDController m_pidController;
  private AbsoluteEncoder m_encoder;

  public SASubsystem() {
 TalonFXConfiguration configs = new TalonFXConfiguration();

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.05; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 1.0; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
    //configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 9;
    configs.Voltage.PeakReverseVoltage = -9;
    m_shooteronemotor.getConfigurator().apply(configs);  
    m_shootertwomotor.getConfigurator().apply(configs);  
  
    //configuring the arm motors
    m_armonemotor.restoreFactoryDefaults();
    m_armtwomotor.restoreFactoryDefaults();
    m_armonemotor.setInverted(true);
    //m_armtwomotor.setInverted(true);

    m_armtwomotor.follow(m_armonemotor, true);
    m_pidController = m_armonemotor.getPIDController();

    m_encoder = m_armonemotor.getAbsoluteEncoder(Type.kDutyCycle);
    m_encoder.setPositionConversionFactor(360);
    m_encoder.setZeroOffset(282);
    m_encoder.setInverted(true);
    m_pidController.setFeedbackDevice(m_encoder);
    //m_pidController.setP(0);
    m_pidController.setP(0.01);
    m_pidController.setI(0.00000);
    m_pidController.setD(0.3);
   // m_pidController.setIZone(kIz);
    m_pidController.setFF(0);
    m_pidController.setOutputRange(-0.25, 0.5);
    kVG = 0.8;
    m_armonemotor.setSmartCurrentLimit(40, 80);
    m_armtwomotor.setSmartCurrentLimit(40, 80);
    m_armonemotor.setSoftLimit(SoftLimitDirection.kReverse,160);
    m_armonemotor.setSoftLimit(SoftLimitDirection.kForward,180);
    m_armonemotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_armonemotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_armonemotor.burnFlash();
    m_armtwomotor.burnFlash();
    //desiredAngle = null;
  }
@Override
public void periodic(){
  SmartDashboard.putNumber("armangle", getArmAngle());
  double gravff = Math.sin(Math.toRadians(m_encoder.getPosition()))*kVG;
  if (desiredAngle<=m_encoder.getPosition()) {gravff = gravff*-1;}
  m_pidController.setReference(desiredAngle, CANSparkMax.ControlType.kPosition,0, gravff);
}
  /**
   * Example command factory method.
   *
   * @return a command
   */
public void setP(double pgain) {
m_pidController.setP(pgain);
}

public void shoot(double rps) {
  m_shooteronemotor.setControl(m_voltageVelocity.withVelocity(rps));
  m_shootertwomotor.setControl(m_voltageVelocity.withVelocity(rps));

  }
 
public void stop() {
  m_shooteronemotor.setControl(m_voltageVelocity.withVelocity(0));
  m_shootertwomotor.setControl(m_voltageVelocity.withVelocity(0));
    desiredAngle = 105;
     // m_pidController.setReference(100, CANSparkMax.ControlType.kPosition);

}
public void zAim(double angle) {
  double finalangle;
  //finalangle = angle / 360;
 //m_pidController.setReference(angle, CANSparkMax.ControlType.kPosition);
desiredAngle = angle;
  }

  public double getShooterSpeed() {
    var supplyVelocitySignal = m_shooteronemotor.getVelocity();
    return supplyVelocitySignal.getValue();
  }
    public double getArmAngle() {
    double armangle = m_encoder.getPosition();
    return armangle;


  }
}