package TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.RobotConfig;

@TeleOp
public class Teleop extends RobotConfig {
    
    private double drive;
    private double strafe;
    private double turning;

    private double turnOfArm;    
    
    @Override
    public void runOpMode() {
        
        this.initializeHardware();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Status", "Running");
            telemetry.update();

            drive = squareInputWithSign(gamepad1.left_stick_y);
            strafe = squareInputWithSign(gamepad1.left_stick_x);
            turning = squareInputWithSign(-gamepad1.right_stick_x);
            
            frontleftdrive.setPower((drive + strafe - turning));
            frontrightdrive.setPower((drive - strafe + turning));
            backleftdrive.setPower((drive - strafe - turning));
            backrightdrive.setPower((drive + strafe + turning));

            if (gamepad2.dpad_up){
                armUp();
            } else if (gamepad2.dpad_down) {
                armDown();
            } 
            
            if (gamepad2.right_bumper) {
                this.servoOpen();
            } 
            else if (gamepad2.left_bumper) {
                this.servoClose();
            }
          
            if (gamepad2.b) {
                this.rotateServo(45);
            }
            if (gamepad2.y) {
            this.rotateServo(-90);
            }  
            if (gamepad2.a) {
            this.rotateServo(180);
            }
            
            //launcher  
            if(gamepad1.a) {
               this.launchRest();
            }
            if(gamepad1.y){
                this.launch();
            }
        }
    }
   
    private double squareInputWithSign(double input) {
        double output = input * input;
        if (input < 0) {
            output *= -1;
        }
        return output;
    }    
}

