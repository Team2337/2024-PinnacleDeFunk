package frc.robot.subsystems;

import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.nerdyfiles.utilities.CTREUtils;

public class Shooter extends SubsystemBase {

    private TalonFX shooterMotorTopLeft = new TalonFX(40, "Upper");
    private TalonFX shooterMotorTopRight = new TalonFX(41, "Upper");
    private TalonFX shooterMotorBottomLeft = new TalonFX(42, "Upper");
    private TalonFX shooterMotorBottomRight = new TalonFX(43, "Upper");
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    public boolean shooterUpToSpeed = false;
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0);
    private Supplier<String> allianceColor;
    private Supplier<Double> poseY;
    private double speakerCenterY, topLeftVelo, topRightVelo, bottomLeftVelo, bottomRightVelo;
    private double delayCounter = 0;
    private boolean clockwiseRotation = false;
    

    private ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
    private GenericEntry leftShooterVelocityFromDash = shooterTab
        .add("ShooterVelocity", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("Min", -6000, "Max", 6000))
        .getEntry();
    private GenericEntry upDownPercentDifferenceFromDash = shooterTab
        .add("UpDownPercentDifference", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("Min", -100, "Max", 100))
        .getEntry();
    private GenericEntry leftRightPercentDifferenceFromDash = shooterTab
        .add("LeftRightPercentDifference", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("Min", -100, "Max", 100))
        .getEntry();

    //Put data onto shooter tab
    private GenericEntry leftDashNum = shooterTab.add("Left Shooter Speed", 0).getEntry();
    private GenericEntry rightDashNum = shooterTab.add("Right Shooter Speed", 0).getEntry();
    private GenericEntry topLeftVelocity = shooterTab.add("Top Left Velocity", 0).getEntry();
    private GenericEntry topRightVelocity = shooterTab.add("Top Right Velocity", 0).getEntry();
    private GenericEntry bottomLeftVelocity = shooterTab.add("Bottom Left Velocity", 0).getEntry();
    private GenericEntry bottomRightVelocity = shooterTab.add("Bottom Right Velocity", 0).getEntry();
    private GenericEntry topLeftTemp = shooterTab.add("Top Left Temp", 0).getEntry();
    private GenericEntry topRightTemp = shooterTab.add("Top Right Temp", 0).getEntry();
    private GenericEntry bottomLeftTemp = shooterTab.add("Bottom Left Temp", 0).getEntry();
    private GenericEntry bottomRightTemp = shooterTab.add("Bottom Right Temp", 0).getEntry();

    private double leftVelocityFromDash, rightVelocityFromDash, globalVelocity = 0;
    //private double leftVelocityFromDash, rightVelocityFromDash = 0;
    private double shooterKP = 0.8;
    private double shooterKI = 0;
    private double shooterKD = 0;
    private double shooterKV = 0.12;


    public Shooter(Supplier<String> allianceColor, Supplier<Double> poseY) {
        this.allianceColor = allianceColor;
        this.poseY = poseY;

        var setShooterMotorTopLeftToDefault = new TalonFXConfiguration();
        shooterMotorTopLeft.getConfigurator().apply(setShooterMotorTopLeftToDefault);
        var setShooterMotorTopRightToDefault = new TalonFXConfiguration();
        shooterMotorTopLeft.getConfigurator().apply(setShooterMotorTopRightToDefault);
        var setShooterMotorBottomLeftToDefault = new TalonFXConfiguration();
        shooterMotorTopLeft.getConfigurator().apply(setShooterMotorBottomLeftToDefault);
        var setShooterMotorBottomRightToDefault = new TalonFXConfiguration();
        shooterMotorTopLeft.getConfigurator().apply(setShooterMotorBottomRightToDefault);

        ClosedLoopRampsConfigs rampsConfigs = new ClosedLoopRampsConfigs();
        rampsConfigs.withVoltageClosedLoopRampPeriod(0.5);
        
        TalonFXConfiguration topLeftMotorConfig = new TalonFXConfiguration();
        topLeftMotorConfig.withCurrentLimits(CTREUtils.setShooterCurrentLimit());
        topLeftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        topLeftMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        topLeftMotorConfig.Slot0.kP = shooterKP;
        topLeftMotorConfig.Slot0.kI = shooterKI;
        topLeftMotorConfig.Slot0.kD = shooterKD;
        topLeftMotorConfig.Slot0.kV = shooterKV;
        topLeftMotorConfig.Voltage.PeakForwardVoltage = 12;
        topLeftMotorConfig.Voltage.PeakReverseVoltage = -12;
        topLeftMotorConfig.Slot1.kP = 0.5;
        topLeftMotorConfig.Slot1.kI = 0.1;
        topLeftMotorConfig.Slot1.kD = 0.001;
        topLeftMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        topLeftMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        topLeftMotorConfig.withClosedLoopRamps(rampsConfigs);
        
        shooterMotorTopLeft.getConfigurator().apply(topLeftMotorConfig);
        
        TalonFXConfiguration topRightMotorConfig = new TalonFXConfiguration();
        topRightMotorConfig.withCurrentLimits(CTREUtils.setShooterCurrentLimit());
        topRightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        topRightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        topRightMotorConfig.Slot0.kP = shooterKP;
        topRightMotorConfig.Slot0.kI = shooterKI;
        topRightMotorConfig.Slot0.kD = shooterKD;
        topRightMotorConfig.Slot0.kV = shooterKV;
        topRightMotorConfig.Voltage.PeakForwardVoltage = 12;
        topRightMotorConfig.Voltage.PeakReverseVoltage = -12;
        topRightMotorConfig.Slot1.kP = 0.5;
        topRightMotorConfig.Slot1.kI = 0.1;
        topRightMotorConfig.Slot1.kD = 0.001;
        topRightMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        topRightMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        topRightMotorConfig.withClosedLoopRamps(rampsConfigs);

        shooterMotorTopRight.getConfigurator().apply(topRightMotorConfig);

        TalonFXConfiguration bottomLeftMotorConfig = new TalonFXConfiguration();
        bottomLeftMotorConfig.withCurrentLimits(CTREUtils.setShooterCurrentLimit());
        bottomLeftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        bottomLeftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        bottomLeftMotorConfig.Slot0.kP = shooterKP;
        bottomLeftMotorConfig.Slot0.kI = shooterKI;
        bottomLeftMotorConfig.Slot0.kD = shooterKD;
        bottomLeftMotorConfig.Slot0.kV = shooterKV;
        bottomLeftMotorConfig.Voltage.PeakForwardVoltage = 12;
        bottomLeftMotorConfig.Voltage.PeakReverseVoltage = -12;
        bottomLeftMotorConfig.Slot1.kP = 0.5;
        bottomLeftMotorConfig.Slot1.kI = 0.1;
        bottomLeftMotorConfig.Slot1.kD = 0.001;
        bottomLeftMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        bottomLeftMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        bottomLeftMotorConfig.withClosedLoopRamps(rampsConfigs);

        shooterMotorBottomLeft.getConfigurator().apply(bottomLeftMotorConfig);

       TalonFXConfiguration bottomRightMotorConfig = new TalonFXConfiguration();
        bottomRightMotorConfig.withCurrentLimits(CTREUtils.setShooterCurrentLimit());
        bottomRightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        bottomRightMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        bottomRightMotorConfig.Slot0.kP = shooterKP;
        bottomRightMotorConfig.Slot0.kI = shooterKI;
        bottomRightMotorConfig.Slot0.kD = shooterKD;
        bottomRightMotorConfig.Slot0.kV = shooterKV;
        bottomRightMotorConfig.Voltage.PeakForwardVoltage = 12;
        bottomRightMotorConfig.Voltage.PeakReverseVoltage = -12;
        bottomRightMotorConfig.Slot1.kP = 0.5;
        bottomRightMotorConfig.Slot1.kI = 0.1;
        bottomRightMotorConfig.Slot1.kD = 0.001;
        bottomRightMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        bottomRightMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        bottomRightMotorConfig.withClosedLoopRamps(rampsConfigs);
        
        shooterMotorBottomRight.getConfigurator().apply(bottomRightMotorConfig);
    }

    public void setShooterDutyCycleZero() {
        shooterMotorTopLeft.setControl(dutyCycle.withOutput(0));
        shooterMotorBottomLeft.setControl(dutyCycle.withOutput(0));
        shooterMotorTopRight.setControl(dutyCycle.withOutput(0));
        shooterMotorBottomRight.setControl(dutyCycle.withOutput(0));
    }


    //Individual
    public void setTopLeftShooterVelocity(double velocity) {
        shooterMotorTopLeft.setControl(velocityVoltage.withVelocity(velocity));
        globalVelocity = velocity;
    }

    public void setTopRightShooterVelocity(double velocity) {
        shooterMotorTopRight.setControl(velocityVoltage.withVelocity(velocity));
        globalVelocity = velocity;
    }

    public void setBottomLeftShooterVelocity(double velocity) {
        shooterMotorBottomLeft.setControl(velocityVoltage.withVelocity(velocity));
        globalVelocity = velocity;
    }

    public void setBottomRightShooterVelocity(double velocity) {
        shooterMotorBottomRight.setControl(velocityVoltage.withVelocity(velocity));
        globalVelocity = velocity;
    }

    //Get temps
    public double getTopLeftMotorTemp() {
        return shooterMotorTopLeft.getDeviceTemp().getValueAsDouble();
    }

    public double getTopRightMotorTemp() {
        return shooterMotorTopRight.getDeviceTemp().getValueAsDouble();
    }

    public double getBottomLeftMotorTemp() {
        return shooterMotorBottomLeft.getDeviceTemp().getValueAsDouble();
    }

    public double getBottomRightMotorTemp() {
        return shooterMotorBottomRight.getDeviceTemp().getValueAsDouble();
    }

    public boolean getShooterUpToSpeed() {
        return shooterUpToSpeed;
    }

    public void checkShooterUpToSpeed() {
        
        if ((shooterMotorTopLeft.getVelocity().getValueAsDouble() >= (globalVelocity * 0.97)) && (globalVelocity > 5) ) {//&& (delayCounter >= 20)) {
            shooterUpToSpeed = true;
        // } else if (delayCounter < 20) {
        //     delayCounter++;
        } else {
            delayCounter = 0;
            shooterUpToSpeed = false;
        }
    }

    //Change both at the same time
    public void setAllPercentVelocityByPercent(double upDownVelo, double leftRightVelo, double velocity) {
        //double velocity = readLeftShooterVelocity();
        setTopLeftShooterVelocity(velocity);
        setTopRightShooterVelocity(velocity*((100-leftRightVelo)/100));
        setBottomLeftShooterVelocity(velocity*((100-upDownVelo)/100));
        setBottomRightShooterVelocity(velocity*(((100-upDownVelo)/100))*((100-leftRightVelo)/100));
    }

    public void setAllPercentVelocity() {
        double maxVelocity = Constants.Shooter.SHOOTER_MAX_VELOCITY;
        if (clockwiseRotation) {
            topLeftVelo = maxVelocity;
            bottomLeftVelo = maxVelocity + Constants.Shooter.SHOOTER_BOTTOM_DIFF;
            topRightVelo = maxVelocity + Constants.Shooter.SHOOTER_LEFTRIGHT_DIFF;
            bottomRightVelo = maxVelocity + Constants.Shooter.SHOOTER_LEFTRIGHT_DIFF + Constants.Shooter.SHOOTER_BOTTOM_DIFF;
        } else {
            topRightVelo = maxVelocity;
            bottomRightVelo = maxVelocity + Constants.Shooter.SHOOTER_BOTTOM_DIFF;
            topLeftVelo = maxVelocity + Constants.Shooter.SHOOTER_LEFTRIGHT_DIFF;
            bottomLeftVelo = maxVelocity + Constants.Shooter.SHOOTER_LEFTRIGHT_DIFF + Constants.Shooter.SHOOTER_BOTTOM_DIFF;
        }
        setTopLeftShooterVelocity(topLeftVelo);
        setTopRightShooterVelocity(topRightVelo);
        setBottomLeftShooterVelocity(bottomLeftVelo);
        setBottomRightShooterVelocity(bottomRightVelo);
    }

    public void setAllPercentVelocityAmp() {
        double maxVelocity = Constants.Shooter.SHOOTER_MAX_VELOCITY;

        topLeftVelo = maxVelocity;
        bottomLeftVelo = maxVelocity + Constants.Shooter.SHOOTER_BOTTOM_DIFF;
        topRightVelo = maxVelocity + Constants.Shooter.SHOOTER_LEFTRIGHT_DIFF;
        bottomRightVelo = maxVelocity + Constants.Shooter.SHOOTER_LEFTRIGHT_DIFF + Constants.Shooter.SHOOTER_BOTTOM_DIFF;

        setTopLeftShooterVelocity(topLeftVelo);
        setTopRightShooterVelocity(topRightVelo);
        setBottomLeftShooterVelocity(bottomLeftVelo);
        setBottomRightShooterVelocity(bottomRightVelo);
    }

    public void log() {
        if (Constants.DashboardLogging.SHOOTER) {
            SmartDashboard.putNumber("Shooter/Top Left Motor Temperature", getTopLeftMotorTemp());
            SmartDashboard.putNumber("Shooter/Top Right Motor Temperature", getTopRightMotorTemp());
            SmartDashboard.putNumber("Shooter/Bottom Left Motor Temperature", getBottomLeftMotorTemp());
            SmartDashboard.putNumber("Shooter/Bottom Right Motor Temperature", getBottomRightMotorTemp());
            SmartDashboard.putNumber("Shooter/Top Right Error", shooterMotorTopRight.getClosedLoopError().getValueAsDouble());
            SmartDashboard.putNumber("Shooter/Top Left Motor Amp", shooterMotorTopRight.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Shooter/Top Right Motor Amp", shooterMotorTopLeft.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Shooter/Bottom Left Motor Amp", shooterMotorBottomLeft.getStatorCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Shooter/Bottom Right Motor Amp", shooterMotorBottomRight.getStatorCurrent().getValueAsDouble());
        }
        //Put Velocity Numbers Onto Shooter Tab
        topLeftVelocity.setDouble(shooterMotorTopLeft.getVelocity().getValueAsDouble());
        topRightVelocity.setDouble(shooterMotorTopRight.getVelocity().getValueAsDouble());
        bottomLeftVelocity.setDouble(shooterMotorBottomLeft.getVelocity().getValueAsDouble());
        bottomRightVelocity.setDouble(shooterMotorBottomRight.getVelocity().getValueAsDouble());
        //Put Temp Numbers Onto Shooter Tab
        topLeftTemp.setDouble(shooterMotorTopLeft.getDeviceTemp().getValueAsDouble());
        topRightTemp.setDouble(shooterMotorTopRight.getDeviceTemp().getValueAsDouble());
        bottomLeftTemp.setDouble(shooterMotorBottomLeft.getDeviceTemp().getValueAsDouble());
        bottomRightTemp.setDouble(shooterMotorBottomRight.getDeviceTemp().getValueAsDouble());

        
    }

    public double getSpeakerCenter() {
        if (allianceColor.get() == "red") {
            speakerCenterY = Constants.FieldElements.redSpeakerCenter.getY();
        } else {
            speakerCenterY = Constants.FieldElements.blueSpeakerCenter.getY();
        }

        return speakerCenterY;
    }

    @Override
    public void periodic() {
        super.periodic();
        log();
        leftVelocityFromDash = leftShooterVelocityFromDash.getDouble(0);
        /*setLeftShooterVelocity(leftVelocityFromDash);
        rightVelocityFromDash = rightShooterVelocityFromDash.getDouble(0);
        setRightShooterVelocity(rightVelocityFromDash);*/

        //Different Options For Slider Tests
        //setTopBottomPrecentVelocity(upDownPercentDifferenceFromDash.getDouble(0));
        //setLeftRightPrecentVelocity(leftRightPercentDifferenceFromDash.getDouble(0));
        //setAllPercentVelocity(upDownPercentDifferenceFromDash.getDouble(0), leftRightPercentDifferenceFromDash.getDouble(0));


        checkShooterUpToSpeed();
        if (poseY.get() < getSpeakerCenter()) {
            clockwiseRotation = false;
        } else {
            clockwiseRotation = true;
        }
       
    }
}
