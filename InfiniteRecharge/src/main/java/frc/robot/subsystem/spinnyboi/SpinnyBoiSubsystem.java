package frc.robot.subsystem.spinnyboi;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketSubsystem;
import frc.robot.utils.talonutils.MotorUtils;

public class SpinnyBoiSubsystem extends BitBucketSubsystem {
    public enum SpinnyBoiState {
        Clockwise, CounterClockwise, Off;
    }

    private SpinnyBoiState state = SpinnyBoiState.Off;

    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    WPI_TalonSRX motor;

    private double spinTime;

    public SpinnyBoiSubsystem(Config config) {
        super(config);
    }

    public void initialize() {
        super.initialize();
        motor = MotorUtils.makeSRX(config.spinnyboi.spinner);

    }

    public double distanceCalculator(Color c1, Color c2) {
        return Math
                .sqrt(Math.pow(c1.red - c2.red, 2) + Math.pow(c1.green - c2.green, 2) + Math.pow(c1.blue - c2.blue, 2));
    }


    public Color nearestColor(Color a){
        double dist[] = {distanceCalculator(a, SpinnyBoiConstants.yellow),distanceCalculator(a, SpinnyBoiConstants.red),distanceCalculator(a, SpinnyBoiConstants.green),distanceCalculator(a, SpinnyBoiConstants.blue)};
        double index = 0;
        double min = dist[0];

        for (int i = 1; i < dist.length; i++) {
            if (min > dist[i]) {
                min = dist[i];
                index = i;
            }
        }

        if(index == 0){
            return SpinnyBoiConstants.yellow;
        }
        else if(index == 1){
            return SpinnyBoiConstants.red;
        }
        else if(index == 2){
            return SpinnyBoiConstants.green;
        }
        else{
            return SpinnyBoiConstants.blue; 
        }

    }

    // 123456789101112131415161718192021222324252627282930313233343536373839404142434445464748495051525354555657585960616263646566676869707172737475767778798081828384858687888990919293949596979899100
    spinnyBoiTargetColor = SmartDashboard.getNumber(SpinnyBoiSubsystem.getName() + "/Dashboard Elevation Target", 0);
    
    public void getDirection(SpinnyBoiColor index){
        int currentColor = index.index;
        int targetColor = index.spinnyBoiTargetColor;
        double indexDifference = targetColor-currentColor; 
        if((indexDifference > 0) && (indexDifference != 3)){
           state = SpinnyBoiState.Clockwise;
           spinTime = indexDifference * SpinnyBoiConstants.SECONDS_PER_ARC;
        }
        else if((indexDifference < 0) && (indexDifference != -3)){
            state = SpinnyBoiState.CounterClockwise;
            spinTime = indexDifference * SpinnyBoiConstants.SECONDS_PER_ARC;

        }
        else if(indexDifference == 3){
            state = SpinnyBoiState.CounterClockwise;
            spinTime = 1 * SpinnyBoiConstants.SECONDS_PER_ARC;
        }
        else if(indexDifference == -3){
            state = SpinnyBoiState.Clockwise;
            spinTime = 1 * SpinnyBoiConstants.SECONDS_PER_ARC;
        }
        else{
            state = SpinnyBoiState.Off;
        }



    }
    public void testInit() {

    }

    public void testPeriodic() {

    }

    public void diagnosticsCheck() {

    }

    public void forward() {
        motor.set(
                SmartDashboard.getNumber(getName() + "/Spinnyboi forward speed", SpinnyBoiConstants.SPINNYBOI_FORWARD));
        SmartDashboard.putString(getName() + "/Spinnyboi state", "forward");
    }

    public void backward() {
        motor.set(-SmartDashboard.getNumber(getName() + "/Spinnyboi backward speed",
                SpinnyBoiConstants.SPINNIYBOI_BACKWARD));
        SmartDashboard.putString(getName() + "/Spinnyboi state", "backward");
    }

    public void off() {
        motor.set(0);
        SmartDashboard.putString(getName() + "/Spinnyboi state", "off");
    }

    @Override
    public void periodic(float deltaTime) {
        updateBaseDashboard();
        switch (state) {

            case Off:
                motor.set(0);
                SmartDashboard.putString(getName() + "/Spinnyboi state", "Not spinning");
                break;

            case Clockwise:
                motor.set(SmartDashboard.getNumber(getName() + "/Intake Speed", SpinnyBoiConstants.SPINNYBOI_FORWARD));
                SmartDashboard.putString(getName() + "/Spinnyboi state", "spinning clockwise");
                break;

            case CounterClockwise:
                motor.set(
                        -SmartDashboard.getNumber(getName() + "/Intake Speed", SpinnyBoiConstants.SPINNIYBOI_BACKWARD));
                SmartDashboard.putString(getName() + "/Spinnyboi state", "spinning counterclockwise");
                break;
        }
        Color detectedColor = m_colorSensor.getColor();

        /**
         * The sensor returns a raw IR value of the infrared light detected.
         */
        double IR = m_colorSensor.getIR();

        /**
         * Open Smart Dashboard or Shuffleboard to see the color detected by the sensor.
         */
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("IR", IR);

        /**
         * In addition to RGB IR values, the color sensor can also return an infrared
         * proximity value. The chip contains an IR led which will emit IR pulses and
         * measure the intensity of the return. When an object is close the value of the
         * proximity will be large (max 2047 with default settings) and will approach
         * zero when the object is far away.
         * 
         * Proximity can be used to roughly approximate the distance of an object or
         * provide a threshold for when an object is close enough to provide accurate
         * color values.
         */
        int proximity = m_colorSensor.getProximity();

        SmartDashboard.putNumber("Proximity", proximity);

    }

    public void rotationControl() {
        // Rotate the wheel 3 - 5 times

        motor.set(ControlMode.MotionMagic, SpinnyBoiConstants.SECONDS_PER_ARC);
    }

    public void manualRotationControl() {
        // In case something goes horribly wrong

        // motor.set()
    }

    public void colorControl() {
        // blue rgb values: 0, 255, 255
        // green rgb values: 0, 255, 0
        // red rgb values: 255, 0, 0
        // yellow rgb values: 255, 255, 0

        // Rotate the wheel to the color specified by the FMS
    }

    @Override
    public void dashboardPeriodic(float deltaTime) {
        // TODO Auto-generated method stub

    }

    public void disable() {
        motor.set(0);
    }

    @Override
    protected void listTalons() {
        talons.add(motor);
    }
}