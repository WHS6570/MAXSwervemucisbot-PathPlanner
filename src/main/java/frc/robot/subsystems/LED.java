package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.OIConstants;

public class LED {
    static Spark blinkin = new Spark(1);


public void setColor(double color){
    blinkin.set(color);
}

}
