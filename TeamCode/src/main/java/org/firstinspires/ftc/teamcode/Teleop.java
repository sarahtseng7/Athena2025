

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="FINAL", group="Linear OpMode")

public class Teleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor outtakeL = null;
    private DcMotor outtakeR = null;
    private DcMotor intake = null;
    private double MAX = 0.5;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        backLeft  = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        outtakeR  = hardwareMap.get(DcMotor.class, "outtake_R");
        outtakeL  = hardwareMap.get(DcMotor.class, "outtake_L");
        outtakeL.setDirection(DcMotor.Direction.FORWARD);
        outtakeR.setDirection(DcMotor.Direction.REVERSE);

        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        //intake
        intake  = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftFrontPower;
            double rightFrontPower;
            double leftBackPower;
            double rightBackPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.left_stick_x;
            double strafe = gamepad1.right_stick_x;
            leftFrontPower    = Range.clip(drive + turn + strafe, -MAX, MAX) ;
            rightFrontPower   = Range.clip(drive - turn - strafe, -MAX, MAX) ;
            leftBackPower    = Range.clip(drive + turn - strafe, -MAX, MAX) ;
            rightBackPower   = Range.clip(drive - turn + strafe, -MAX, MAX) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            frontLeft.setPower(leftFrontPower);
            frontRight.setPower(rightFrontPower);
            backLeft.setPower(leftBackPower);
            backRight.setPower(rightBackPower);

            if (gamepad1.rightBumperWasPressed()) {
                MAX = 0.8;
            }
            if (gamepad1.leftBumperWasPressed()) {
                MAX = 0.4;
            }
            if (gamepad1.right_trigger > 0) {
                MAX = 0.6;
            }
            if (gamepad1.left_trigger > 0) {
                MAX = 0.2;
            }
            if (gamepad1.x) {
                MAX = 1.0;
            }

            double outtake_power = -gamepad2.left_stick_y;
            outtakeR.setPower(outtake_power);
            outtakeL.setPower(outtake_power);
            //output trigger
            if (gamepad2.right_trigger != 0) {
                outtakeR.setPower(gamepad2.right_trigger);
                outtakeL.setPower(gamepad2.right_trigger);
            }

            //input
            double intakePower;
            intakePower = Range.clip(-gamepad2.right_stick_y, -0.7, 0.4) ;
            intake.setPower(intakePower);


            if (gamepad2.left_trigger != 0) {
                intakePower   = Range.clip(gamepad2.left_trigger, -0.7, 0.7) ;
                intake.setPower(intakePower);
            }
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("MAX SPEED", MAX);
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", rightPower);
            telemetry.update();
        }
    }
}
