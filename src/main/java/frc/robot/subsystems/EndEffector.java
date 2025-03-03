package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase 
{
    private static EndEffector instance;
    private TalonFX motor;

    private Canandcolor frontCanandcolor;
    private Canandcolor backCanandcolor;
    
    private EndEffector ()
    {
        motor = new TalonFX(Constants.EndEffector.ID, Constants.CAN_CHAIN);
        config();

        frontCanandcolor = new Canandcolor(Constants.EndEffector.FRONT_CANANDCOLOR_ID);
        backCanandcolor = new Canandcolor(Constants.EndEffector.BACK_CANANDCOLOR_ID);
    }

    private void config ()
    {
        motor.clearStickyFaults();

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = Constants.EndEffector.INVERTED;

        config.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        config.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        config.CurrentLimits.StatorCurrentLimit = Constants.EndEffector.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        motor.getConfigurator().apply(config);
    }

    /**
     * @return rotations per second
     */
    public double getSpeed ()
    {
        return motor.getVelocity().getValueAsDouble();
    }

    /**
     * @param speed rotations per second
     */
    public void setSpeed (double speed)
    {
        motor.setControl(new DutyCycleOut(speed));
    }

    public boolean isBackTriggered ()
    {
        return backCanandcolor.getProximity() < Constants.EndEffector.PROXIMITY_LIMIT_BACK;
    }

    public boolean isFrontTriggered ()
    {
        return frontCanandcolor.getProximity() < Constants.EndEffector.PROXIMITY_LIMIT_FRONT;
    }

    public static EndEffector getInstance ()
    {
        if (instance == null) instance = new EndEffector();
        return instance;
    }
}