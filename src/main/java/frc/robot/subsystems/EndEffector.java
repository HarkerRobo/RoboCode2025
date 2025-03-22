package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase 
{
    private static EndEffector instance;
    private TalonFX mainMotor;
    private TalonFX tuskMotor;

    private Canandcolor frontCanandcolor;
    private Canandcolor backCanandcolor;
    
    private boolean passiveOn;
    private boolean algaeIn;
    private double desiredPosition;
    
    private EndEffector ()
    {
        mainMotor = new TalonFX(Constants.EndEffector.MAIN_ID, Constants.CAN_CHAIN);
        tuskMotor = new TalonFX(Constants.EndEffector.TUSK_ID, Constants.CAN_CHAIN);
    
        frontCanandcolor = new Canandcolor(Constants.EndEffector.FRONT_CANANDCOLOR_ID);
        backCanandcolor = new Canandcolor(Constants.EndEffector.BACK_CANANDCOLOR_ID);

        config();

        passiveOn = true;
        desiredPosition = 0;
        algaeIn = false;
    }

    private void config ()
    {
        mainMotor.clearStickyFaults();
        tuskMotor.clearStickyFaults();

        TalonFXConfiguration mainConfig = new TalonFXConfiguration();

        mainConfig.MotorOutput.Inverted = Constants.EndEffector.MAIN_INVERTED;

        mainConfig.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        mainConfig.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        mainConfig.CurrentLimits.StatorCurrentLimit = Constants.EndEffector.STATOR_CURRENT_LIMIT;
        mainConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        
        mainConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        TalonFXConfiguration tuskConfig = new TalonFXConfiguration();

        tuskConfig.MotorOutput.Inverted = Constants.EndEffector.TUSK_INVERTED;

        tuskConfig.Feedback.SensorToMechanismRatio = Constants.EndEffector.TUSK_GEAR_RATIO;

        tuskConfig.Slot0.kP = Constants.EndEffector.kP;
        tuskConfig.Slot0.kI = Constants.EndEffector.kI;
        tuskConfig.Slot0.kD = Constants.EndEffector.kD;

        tuskConfig.Slot0.kG = Constants.EndEffector.kG;

        tuskConfig.Voltage.PeakForwardVoltage = Constants.MAX_VOLTAGE;
        tuskConfig.Voltage.PeakReverseVoltage = -Constants.MAX_VOLTAGE;

        tuskConfig.CurrentLimits.StatorCurrentLimit = Constants.EndEffector.STATOR_CURRENT_LIMIT;
        tuskConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        
        tuskConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        tuskMotor.getConfigurator().apply(tuskConfig);
        
        CanandcolorSettings canandcolorSettings = new CanandcolorSettings();
        canandcolorSettings.setProximityFramePeriod(Constants.EndEffector.CANANDCOLOR_PROXIMITY_TIME_PERIOD);
        
        frontCanandcolor.setSettings(canandcolorSettings);
        backCanandcolor.setSettings(canandcolorSettings);

    }

    /**
     * @return rotations per second
     */
    public double getMainSpeed ()
    {
        return mainMotor.getVelocity().getValueAsDouble();
    }

    /**
     * @param speed rotations per second
     */
    public void setMainSpeed (double speed)
    {
        mainMotor.setControl(new DutyCycleOut(speed));
    }

    public boolean isBackTriggered ()
    {
        return backCanandcolor.getProximity() < Constants.EndEffector.PROXIMITY_LIMIT_BACK;
    }

    public boolean isFrontTriggered ()
    {
        return frontCanandcolor.getProximity() < Constants.EndEffector.PROXIMITY_LIMIT_FRONT;
    }

    public boolean isTuskStalling()
    {
        return tuskMotor.getStatorCurrent().getValueAsDouble() >= Constants.EndEffector.TUSK_STALLING_CURRENT;
    }

    public boolean isMainStalling()
    {
        return Math.abs(mainMotor.getStatorCurrent().getValueAsDouble()) >= Constants.EndEffector.MAIN_STALLING_CURRENT;
    }

    public double getMainMotorCurrent()
    {
        return mainMotor.getStatorCurrent().getValueAsDouble();
    }

    public void setTuskVoltage(double power)
    {
        tuskMotor.setControl(new VoltageOut(power));
    }

    public void setTuskSensorPosition(double position)
    {
        tuskMotor.getConfigurator().setPosition(position);
    }

    // pid control
    public void moveToPosition(double desiredPosition)
    {
        this.desiredPosition = desiredPosition;
        tuskMotor.setControl(new PositionVoltage(desiredPosition));
    }

    /**
     * @return rotations
     */
    public double getTuskPosition() 
    {
        return tuskMotor.getPosition().getValueAsDouble();
    }

    public double getTuskDesiredPosition()
    {
        return desiredPosition;
    }
    
    /**
     * @return rotations per second
     */
    public double getTuskVelocity()
    {
        return tuskMotor.getVelocity().getValueAsDouble();
    }

    /**
     * @param power range [-1, 1]
     */
    public void setTuskPower(double power) 
    {
        tuskMotor.setControl(new DutyCycleOut(power));
    }

    public void togglePassive()
    {
        passiveOn = !passiveOn;
    }

    public void setPassive(boolean val)
    {
        passiveOn = val;
    }

    public boolean getPassive ()
    {
        return passiveOn;
    }

    public void setAlgaeIn(boolean val)
    {
        algaeIn = val;
    }

    public boolean algaeIn ()
    {
        return algaeIn;
    }

    public static EndEffector getInstance ()
    {
        if (instance == null) instance = new EndEffector();
        return instance;
    }
}