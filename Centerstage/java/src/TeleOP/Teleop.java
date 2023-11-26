
// package TeleOP;

// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// import org.firstinspires.ftc.teamcode.common.RobotConfig;


// @TeleOp
// public class Teleop extends RobotConfig {
//     private double drive;
//     private double strafe;
//     private double turning;

//     @Override
//     public void runOpMode() {

//         this.initializeHardware();
        

//         telemetry.addData("Status", "Initialized");
//         telemetry.update();

//         waitForStart();

//         while (opModeIsActive()) {

//             telemetry.addData("Status", "Running");
//             telemetry.update();

//             drive = squareInputWithSign(gamepad1.left_stick_y);
//             strafe = squareInputWithSign(gamepad1.left_stick_x);
//             turning = squareInputWithSign(-gamepad1.right_stick_x);

//             frontleftdrive.setPower((drive + strafe - turning));
//             frontrightdrive.setPower((drive - strafe + turning));
//             backleftdrive.setPower((drive - strafe - turning));
//             backrightdrive.setPower((drive + strafe + turning));


//             // if (gamepad2.a) {
//             //     moveToLow();
//             // } 
//             // if (gamepad2.b) {
//             //     moveToMiddle();
//             // } 
//             // if (gamepad2.y) {
//             //     moveToHigh();
//             // } 
//             // if (gamepad2.x) {
//             //     moveToGround();
//             // } 
//             // if (gamepad2.left_trigger == 1.0) {
//             //     moveToStack34();
//             // }


//             // if (gamepad2.left_bumper) {
//             //     this.clawOpen();
//             // } else if (gamepad2.right_bumper) {
//             //     this.clawClose();
//             // }
            
//         }
//     }

//     /**
//      * This method will reduce the sensitivity of the middle position of joystick.
//      *
//      * @param input
//      * @return
//      */
//     private double squareInputWithSign(double input) {
//         double output = input * input;
//         if (input < 0) {
//             output *= -1;
//         }
//         return output;
//     }
// }
