package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public abstract class RobotConfig extends LinearOpMode {
    // drive train motor 5203 series - https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    static final double COUNTS_PER_REV = 537.7;
    static final double WHEEL_DIA_INCHES = 3.77;
    static final int ticks = (int) ((COUNTS_PER_REV)/(WHEEL_DIA_INCHES * Math.PI));
    
    public static final double wheelpower = 0.6;
    public static final double pulleyPower = 0.6;

    protected DcMotor frontleftdrive;
    protected DcMotor frontrightdrive;
    protected DcMotor backleftdrive;
    protected DcMotor backrightdrive;

    protected DcMotor slidePulley;
    protected AnalogInput potentiometer;
    private double currentVoltage;
    
    protected Servo clawLeftServo;
    protected Servo clawRightServo;
    protected Servo balanceServo;
    protected Servo launchServo;
    
    protected BNO055IMU imu;
    // protected SleeveDetection sleeveDetection;
    // protected OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    String webcamName = "Webcam 1";
    
    int armPosition = 0;

    // protected Servo ring;
    protected void initializeHardware() {
        //Doing all the hardware mappings
        frontleftdrive = hardwareMap.get(DcMotor.class, "frontleftdrive");
        frontrightdrive = hardwareMap.get(DcMotor.class, "frontrightdrive");
        backleftdrive = hardwareMap.get(DcMotor.class, "backleftdrive");
        backrightdrive = hardwareMap.get(DcMotor.class, "backrightdrive");
        
        // arm = hardwareMap.get(DcMotor.class, "arm");
        slidePulley = hardwareMap.get(DcMotor.class,"slidepulley");
        potentiometer = hardwareMap.get(AnalogInput.class, "armpotentiometer");
        
        currentVoltage = potentiometer.getVoltage();
        telemetry.addData("Current Voltage from potentiometer", currentVoltage);
        telemetry.update();

        clawLeftServo = hardwareMap.get(Servo.class,"clawleftservo");
        clawRightServo = hardwareMap.get(Servo.class,"clawrightservo");
        balanceServo = hardwareMap.get(Servo.class,"balanceservo");
        launchServo = hardwareMap.get(Servo.class, "launchservo");
       
        frontleftdrive.setDirection(DcMotor.Direction.FORWARD);
        backleftdrive.setDirection(DcMotor.Direction.FORWARD);
        frontrightdrive.setDirection(DcMotor.Direction.REVERSE);
        backrightdrive.setDirection(DcMotor.Direction.REVERSE);
        
        setModeForWheelMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeForWheelMotors(DcMotor.RunMode.RUN_USING_ENCODER);

        initIMU();

        // Telling Driver that Hardware is Configured
        telemetry.addData("Status: ", "Hardware Configured");
        telemetry.update();
    }

    private void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
    }


    protected void setModeForWheelMotors(DcMotor.RunMode runMode) {
        frontleftdrive.setMode(runMode);
        frontrightdrive.setMode(runMode);
        backleftdrive.setMode(runMode);
        backrightdrive.setMode(runMode);
    }

    private void setWheelsPower(double frontLeftDrivePower,  double frontRightDrivePower, double backLeftDrivePower, double backRightDrivePower) {
        frontleftdrive.setPower(frontLeftDrivePower);
        frontrightdrive.setPower(frontRightDrivePower);
        backleftdrive.setPower(backLeftDrivePower);
        backrightdrive.setPower(backRightDrivePower);
    }

    protected void setTargetPositionForWheelMotors(int frontLeftPos, int frontRightPos, int backLeftPos, int backRightPos) {
        frontleftdrive.setTargetPosition(frontLeftPos);
        frontrightdrive.setTargetPosition(frontRightPos);
        backleftdrive.setTargetPosition(backLeftPos);
        backrightdrive.setTargetPosition(backRightPos);
    }

    private void waitUntilMotorsBusy() {
        while (frontleftdrive.isBusy() && frontrightdrive.isBusy() && backleftdrive.isBusy() && backrightdrive.isBusy()) {
//            telemetry.addData("encoder-front-left", frontleftdrive.getCurrentPosition());
//            telemetry.addData("encoder-front-right", frontrightdrive.getCurrentPosition());
//            telemetry.addData("encoder-back-left", backleftdrive.getCurrentPosition());
//            telemetry.addData("encoder-back-right", frontrightdrive.getCurrentPosition());
//            telemetry.update();
            idle();
        }
    }
    
    protected void forward(int distance) {
        distance = distance * ticks;
        setTargetPositionForWheelMotors(-distance, -distance, -distance, -distance);
        setModeForWheelMotors(DcMotor.RunMode.RUN_TO_POSITION);
        backwardPower();
        waitUntilMotorsBusy();
        stopPower();
        sleep(1000);
    }
    
    private void backwardPower() {
        setWheelsPower(-wheelpower,-wheelpower,-wheelpower,-wheelpower);
    }
    
    protected void backward(int distance) {
        distance = distance * ticks;
        setModeForWheelMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeForWheelMotors(DcMotor.RunMode.RUN_USING_ENCODER);
        setTargetPositionForWheelMotors(distance, distance, distance, distance);
        setModeForWheelMotors(DcMotor.RunMode.RUN_TO_POSITION);
        forwardPower();
        waitUntilMotorsBusy();
        stopPower();
        sleep(1000);
    }

    private void forwardPower() {
        setWheelsPower(wheelpower,wheelpower,wheelpower,wheelpower);
    }
    
    protected void turnLeft(int distance) {
        distance = distance * ticks;
        setTargetPositionForWheelMotors(-distance, distance,-distance, distance);
        setModeForWheelMotors(DcMotor.RunMode.RUN_TO_POSITION);
        leftPower();
        waitUntilMotorsBusy();
        stopPower();
        sleep(1000);
    }
    
    private void leftPower() {
        // setWheelsPower(wheelpower,wheelpower,wheelpower,wheelpower);
        setWheelsPower(0.01,1.0,0.01,1.0);
    }
    
     protected void turnRight(int distance) {
         distance = distance * ticks;
         setTargetPositionForWheelMotors(distance, -distance, distance, -distance);
         setModeForWheelMotors(DcMotor.RunMode.RUN_TO_POSITION);
         rightPower();
         waitUntilMotorsBusy();
         stopPower();
         sleep(1000);
     }

     private void rightPower() {
         //setWheelsPower(wheelpower,-wheelpower,wheelpower,-wheelpower);
         setWheelsPower(1.0,0.01,1.0,0.01);
     } 
    
    protected void strafeLeft(int distance) {
        distance = distance * ticks;
        setTargetPositionForWheelMotors(distance,-distance,-distance,distance);
        setModeForWheelMotors(DcMotor.RunMode.RUN_TO_POSITION);
        strafeLeftPower();
        waitUntilMotorsBusy();
        stopPower();        
        sleep(1000);
    }    

    private void strafeLeftPower() {
        //setWheelsPower(wheelpower,-wheelpower,-wheelpower,wheelpower);
        setWheelsPower(0.7,-0.7,-0.7,0.7);
    }
    
     protected void strafeRight(int distance) {
         distance = distance * ticks;
         setTargetPositionForWheelMotors(-distance, distance, distance, -distance);
         setModeForWheelMotors(DcMotor.RunMode.RUN_TO_POSITION);
         strafeRightPower();
         waitUntilMotorsBusy();
         stopPower();         
         sleep(1000);
     }

     private void strafeRightPower() {
         //setWheelsPower(-wheelpower,wheelpower,wheelpower,-wheelpower);
         setWheelsPower(-0.7,0.7,0.7,-0.7);
     }
    
    private void stopPower() {
        setWheelsPower(0,0,0,0);
    }   
    
    // protected void fastForward(int distance, int power) {
    //     distance = distance * ticks;
    //     setTargetPositionForWheelMotors(-distance, -distance, -distance, -distance);
    //     setModeForWheelMotors(DcMotor.RunMode.RUN_TO_POSITION);
    //     forwardFastPower(power); // technically backward but yeah whatever
    //     waitUntilMotorsBusy();
    //     stopPower();
    //     setModeForWheelMotors(DcMotor.RunMode.RUN_USING_ENCODER);
    //     sleep(1000);
    // }
    
    // private void forwardFastPower(double power) {
    //     setWheelsPower(-power,-power,-power,-power);
    // }    

    // private double getAngle() {
    //     Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    //     return orientation.firstAngle;
    // }

    // private void runWithoutEncoders() {
    //     setModeForWheelMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //     setTargetPositionForWheelMotors(0, 0, 0, 0);
    // }
    
    // Example way to open  servo
    protected void servoClose() {
        clawLeftServo.setPosition(0.15);
        clawRightServo.setPosition(0.15);
    }
    
    protected void servoOpen() {
        clawLeftServo.setPosition(-0.75);
        clawRightServo.setPosition(0.75);
    }
    
    //for balanceServo
    protected void rotateServo(int posn) {
        if (0==posn) { //0 degrees
            balanceServo.setPosition(0.3);
        } 
        else if (90==posn) { //90 degrees
            balanceServo.setPosition(0.5);
        } 
        else if (180==posn){ // 180 degrees
            balanceServo.setPosition(0.9);
        } else {
            telemetry.addData("BaServo Position is  ", balanceServo.getPosition());
            telemetry.update();
            balanceServo.setPosition(0.65);
        }
    }
    
    //launcherServo - launch
    protected void launch() {//y
        launchServo.setDirection(Servo.Direction.FORWARD);
        launchServo.setPosition(0.6);
    }    
    
    //launcherServo - rest position
    protected void launchRest() {
        launchServo.setDirection(Servo.Direction.REVERSE);
        launchServo.setPosition(0.0);
    }
    
    protected void armUp(){        
        armPosition = slidePulley.getCurrentPosition();        
        slidePulley.setTargetPosition(armPosition + 100);
        slidePulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidePulley.setPower(0.2);
    }
    
    protected void armDown() {
        armPosition = slidePulley.getCurrentPosition();
        slidePulley.setTargetPosition(armPosition - 20);        
        slidePulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidePulley.setPower(-0.4);
    }
    
    protected void autoArmUp(){        
        servoClose();
        
        slidePulley.setDirection(DcMotor.Direction.FORWARD);
        slidePulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidePulley.setTargetPosition(slidePulley.getCurrentPosition()+ 750);
        slidePulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidePulley.setPower(-0.2);        
       
        while (slidePulley.isBusy()) {
            idle();
        }
        sleep(1000);
     
        servoOpen();
        
        sleep(2000);
    }
    
    protected void autoArmDown() {        
        slidePulley.setTargetPosition(50);
        slidePulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidePulley.setPower(-0.1);
        while (slidePulley.isBusy()) {
            idle();
        }
    }
}

