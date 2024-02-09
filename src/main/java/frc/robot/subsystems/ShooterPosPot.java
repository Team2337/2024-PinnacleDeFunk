package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.CTREUtils;

public class ShooterPosPot extends PIDSubsystem {
    private TalonFX shootPosPotMotor = new TalonFX(54); 

    
    AnalogInput input = new AnalogInput(4);
    AnalogPotentiometer pot = new AnalogPotentiometer(input, 51.6, 1.6);

    CommandXboxController operatorJoystick;

    double shooterPotMaxSetPoint = 10;
    double shooterPotMinSetPoint = 1.6;

    public ShooterPosPot(CommandXboxController operatorJoystick) {
        super(new PIDController(0.1, 0.0, 0.0001));
        this.operatorJoystick = operatorJoystick;
        getController().setTolerance(2.0);
        setSetpoint(pot.get());
        enable();

        var setShootPosPotMotorToDefault = new TalonFXConfiguration();
        shootPosPotMotor.getConfigurator().apply(setShootPosPotMotorToDefault);

        TalonFXConfiguration shootPosPotMotorConfig = new TalonFXConfiguration();
        shootPosPotMotorConfig.withCurrentLimits(CTREUtils.setDefaultCurrentLimit());
        shootPosPotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shootPosPotMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shootPosPotMotorConfig.Voltage.PeakForwardVoltage = 8;
        shootPosPotMotorConfig.Voltage.PeakReverseVoltage = -8;

        shootPosPotMotor.setSafetyEnabled(false);
        shootPosPotMotor.getConfigurator().apply(shootPosPotMotorConfig);

    }

    // public void setShooterPositionPoint(double setPoint) {
    //         if (setPoint < climberMinSetPoint) {
    //             setPoint = climberMinSetPoint;
    //         } else if (setPoint > climberMaxSetPoint) {
    //             setPoint = climberMaxSetPoint;
    //         }
    //         this.setSetpoint(setPoint);
    // }

    public void enablePID(boolean override) {
        if (override) {
            enable();
        } else {
            disable();
        }
    }
   
    public void getSettedSetPoint() {
        setSetpoint(pot.get());
    }

    public void log() {
        if (Constants.DashboardLogging.SHOOTERPOT) {
            
        }
    }

    public void initialize() {
        setSetpoint(pot.get());
    }

    @Override
    public void periodic() {
        super.periodic();
        log();
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        if (output > Constants.ShooterPosPot.SHOOTERPOT_MAX_PID_SPEED) {
            output  = Constants.ShooterPosPot.SHOOTERPOT_MAX_PID_SPEED;
        } else if (output < -Constants.ShooterPosPot.SHOOTERPOT_MAX_PID_SPEED) {
            output = -Constants.ShooterPosPot.SHOOTERPOT_MAX_PID_SPEED;
        }
        // setClimbSpeed(output);
        SmartDashboard.putNumber("Climb Output", output);
    }

    @Override
    protected double getMeasurement() {
       return pot.get();
    }

   
}
