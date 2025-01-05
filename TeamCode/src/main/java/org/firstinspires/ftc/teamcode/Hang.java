package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Hang", group="Pushbot")
//@Disabled
public class Hang extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot = new Hardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double GEAR_RATIO = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEAR_RATIO) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    //static final double SLIDE_INCHES = (COUNTS_PER_MOTOR_REV * 1) /(5.125);
    static final double DRIVE_SPEED = 1.0;
    static final double TURN_SPEED = 1.0;
    static final double SLIDE_SPEED = 0.05;
    static final double timeoutS = 10;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.left_front_drv_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drv_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_back_drv_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_back_drv_Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.left_front_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_front_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_back_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_back_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.hang_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                robot.left_front_drv_Motor.getCurrentPosition(),
                robot.right_front_drv_Motor.getCurrentPosition(),
                robot.left_back_drv_Motor.getCurrentPosition(),
                robot.right_back_drv_Motor.getCurrentPosition()
        );
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //come down
        // Step 3:  Drive Backwards for 1 Second
        //robot.hang_motor.setPower(-.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //WHERE ACTUAL CODE GOES
        robot.claw2.setPosition(0.9);
        robot.arm.setPosition(0.3);
        robot.basket.setPosition(0.23);
        drive(23.2);
        strafe(-16);
        drive(2.5);
        vSlide(0.5,14.5);
        drive(0.5,5.3);
        //vSlide(0.4,-2.5);
        sleep(1000);
        turn(2);
        drive(0.2,-6);
        sleep(500);
        //turn(2);

        //after hang, move on to get another sample
        drive(-6);
        strafe(47.3);
        drive(4);
        robot.arm.setPosition(1.0);
        robot.claw2.setPosition(0.47);
        sleep(1000);
        robot.claw2.setPosition(0.32);
        sleep(1000);
        robot.arm.setPosition(0.3);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

    public void accelerate(double speed, double targetPosition) {

        if (robot.left_back_drv_Motor.getPower() < speed) {
            while (runtime.seconds() < 0.8) {
                //double sec = Math.ceil(runtime.seconds()*5);
                double power = runtime.seconds() + Math.abs(speed-0.8);
                robot.left_front_drv_Motor.setPower(power);
                robot.right_front_drv_Motor.setPower(power);
                robot.left_back_drv_Motor.setPower(power);
                robot.right_back_drv_Motor.setPower(power);
            }
        }
        if (robot.left_back_drv_Motor.getPower() == speed) {
            while (robot.left_back_drv_Motor.getCurrentPosition() > targetPosition*5/6) {
                //double sec = Math.ceil(runtime.seconds() * 5);
                double power = (targetPosition - robot.left_back_drv_Motor.getCurrentPosition()) / 30;
                robot.left_front_drv_Motor.setPower(power);
                robot.right_front_drv_Motor.setPower(power);
                robot.left_back_drv_Motor.setPower(power);
                robot.right_back_drv_Motor.setPower(power);
            }
        }
        /*if (robot.left_back_drv_Motor.getPower() == speed) {
            while (runtime.seconds() > time-0.2) {
                double time = (targetPosition - robot.left_front_drv_Motor.getCurrentPosition())/24;
                //double sec = Math.ceil(runtime.seconds() * 5);
                double power = time / 5;
                robot.left_front_drv_Motor.setPower(power);
                robot.right_front_drv_Motor.setPower(power);
                robot.left_back_drv_Motor.setPower(power);
                robot.right_back_drv_Motor.setPower(power);
            }
        }

         */
    }
    /*
        public void deccelerate(int inches) {
            double targetDistance = inches * COUNTS_PER_INCH;
            if (robot.left_back_drv_Motor.isBusy()) {
                while (robot.left_back_drv_Motor.getCurrentPosition() > (targetDistance*0.85)) {
                    double distance = Math.ceil((targetDistance-robot.left_back_drv_Motor.getCurrentPosition()));
                    double power = distance/10;
                    robot.left_front_drv_Motor.setPower(power);
                    robot.right_front_drv_Motor.setPower(power);
                    robot.left_back_drv_Motor.setPower(power);
                    robot.right_back_drv_Motor.setPower(power);
                }
            }
        }
        */
    public void drive(double speed, double Inches) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.left_front_drv_Motor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.right_front_drv_Motor.getCurrentPosition() + (int) (-Inches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.left_back_drv_Motor.getCurrentPosition() + (int) (-Inches * COUNTS_PER_INCH);
            newRightBackTarget = robot.right_back_drv_Motor.getCurrentPosition() + (int) (-Inches * COUNTS_PER_INCH);
            robot.left_front_drv_Motor.setTargetPosition(newLeftFrontTarget);
            robot.right_front_drv_Motor.setTargetPosition(newRightFrontTarget);
            robot.left_back_drv_Motor.setTargetPosition(newLeftBackTarget);
            robot.right_back_drv_Motor.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.left_front_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_front_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left_back_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_back_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            //accelerate(DRIVE_SPEED,newLeftBackTarget);

            robot.left_front_drv_Motor.setPower(Math.abs(speed));
            robot.right_front_drv_Motor.setPower(Math.abs(speed));
            robot.left_back_drv_Motor.setPower(Math.abs(speed));
            robot.right_back_drv_Motor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.left_front_drv_Motor.isBusy() && robot.right_front_drv_Motor.isBusy() && robot.right_back_drv_Motor.isBusy() && robot.left_back_drv_Motor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.left_front_drv_Motor.getCurrentPosition(),
                        robot.right_front_drv_Motor.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;
            robot.left_front_drv_Motor.setPower(0);
            robot.right_front_drv_Motor.setPower(0);
            robot.left_back_drv_Motor.setPower(0);
            robot.right_back_drv_Motor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.left_front_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_front_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.left_back_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_back_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public void drive(double Inches) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.left_front_drv_Motor.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.right_front_drv_Motor.getCurrentPosition() + (int) (-Inches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.left_back_drv_Motor.getCurrentPosition() + (int) (-Inches * COUNTS_PER_INCH);
            newRightBackTarget = robot.right_back_drv_Motor.getCurrentPosition() + (int) (-Inches * COUNTS_PER_INCH);
            robot.left_front_drv_Motor.setTargetPosition(newLeftFrontTarget);
            robot.right_front_drv_Motor.setTargetPosition(newRightFrontTarget);
            robot.left_back_drv_Motor.setTargetPosition(newLeftBackTarget);
            robot.right_back_drv_Motor.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.left_front_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_front_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left_back_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_back_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            //accelerate(DRIVE_SPEED,newLeftBackTarget);

            robot.left_front_drv_Motor.setPower(Math.abs(DRIVE_SPEED));
            robot.right_front_drv_Motor.setPower(Math.abs(DRIVE_SPEED));
            robot.left_back_drv_Motor.setPower(Math.abs(DRIVE_SPEED));
            robot.right_back_drv_Motor.setPower(Math.abs(DRIVE_SPEED));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.left_front_drv_Motor.isBusy() && robot.right_front_drv_Motor.isBusy() && robot.right_back_drv_Motor.isBusy() && robot.left_back_drv_Motor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.left_front_drv_Motor.getCurrentPosition(),
                        robot.right_front_drv_Motor.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;
            robot.left_front_drv_Motor.setPower(0);
            robot.right_front_drv_Motor.setPower(0);
            robot.left_back_drv_Motor.setPower(0);
            robot.right_back_drv_Motor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.left_front_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_front_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.left_back_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_back_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void strafe(double inches) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.left_front_drv_Motor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.right_front_drv_Motor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.left_back_drv_Motor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newRightBackTarget = robot.right_back_drv_Motor.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            robot.left_front_drv_Motor.setTargetPosition(newLeftFrontTarget);
            robot.right_front_drv_Motor.setTargetPosition(newRightFrontTarget);
            robot.left_back_drv_Motor.setTargetPosition(newLeftBackTarget);
            robot.right_back_drv_Motor.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.left_front_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_front_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left_back_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_back_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            //accelerate(DRIVE_SPEED,newLeftBackTarget);
            robot.left_front_drv_Motor.setPower(Math.abs(DRIVE_SPEED));
            robot.right_front_drv_Motor.setPower(Math.abs(DRIVE_SPEED));
            robot.left_back_drv_Motor.setPower(Math.abs(DRIVE_SPEED));
            robot.right_back_drv_Motor.setPower(Math.abs(DRIVE_SPEED));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.left_front_drv_Motor.isBusy() && robot.right_front_drv_Motor.isBusy() && robot.right_back_drv_Motor.isBusy() && robot.left_back_drv_Motor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.left_front_drv_Motor.getCurrentPosition(),
                        robot.right_front_drv_Motor.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;
            robot.left_front_drv_Motor.setPower(0);
            robot.right_front_drv_Motor.setPower(0);
            robot.left_back_drv_Motor.setPower(0);
            robot.right_back_drv_Motor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.left_front_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_front_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.left_back_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_back_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void turn(double degrees) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        degrees = -degrees / 4;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.left_front_drv_Motor.getCurrentPosition() + (int) (-degrees * COUNTS_PER_INCH);
            newRightFrontTarget = robot.right_front_drv_Motor.getCurrentPosition() + (int) (-degrees * COUNTS_PER_INCH);
            newLeftBackTarget = robot.left_back_drv_Motor.getCurrentPosition() + (int) (degrees * COUNTS_PER_INCH);
            newRightBackTarget = robot.right_back_drv_Motor.getCurrentPosition() + (int) (-degrees * COUNTS_PER_INCH);
            robot.left_front_drv_Motor.setTargetPosition(newLeftFrontTarget);
            robot.right_front_drv_Motor.setTargetPosition(newRightFrontTarget);
            robot.left_back_drv_Motor.setTargetPosition(newLeftBackTarget);
            robot.right_back_drv_Motor.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.left_front_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_front_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left_back_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_back_drv_Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.left_front_drv_Motor.setPower(Math.abs(TURN_SPEED));
            robot.right_front_drv_Motor.setPower(Math.abs(TURN_SPEED));
            robot.left_back_drv_Motor.setPower(Math.abs(TURN_SPEED));
            robot.right_back_drv_Motor.setPower(Math.abs(TURN_SPEED));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.left_front_drv_Motor.isBusy() && robot.right_front_drv_Motor.isBusy() && robot.right_back_drv_Motor.isBusy() && robot.left_back_drv_Motor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.left_front_drv_Motor.getCurrentPosition(),
                        robot.right_front_drv_Motor.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;
            robot.left_front_drv_Motor.setPower(0);
            robot.right_front_drv_Motor.setPower(0);
            robot.left_back_drv_Motor.setPower(0);
            robot.right_back_drv_Motor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.left_front_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_front_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.left_back_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_back_drv_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void vSlide(double speed, double inches) {
        int vSlideInches;

        if (opModeIsActive()) {
            robot.vSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vSlideInches = robot.vSlideMotor.getCurrentPosition() + (int) (-inches * 97.76);
            robot.vSlideMotor.setTargetPosition(vSlideInches);
            robot.vSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.vSlideMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.vSlideMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d", vSlideInches);
                telemetry.addData("Path2", "Running at %7d",
                        robot.vSlideMotor.getCurrentPosition());

                telemetry.update();
            }

            robot.vSlideMotor.setPower(0);
            robot.vSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    double countWheelDistance() {
        return 0;
    }

    public void hSlide(double speed, double inches) {
        int hSlideTarget;
        if (!opModeIsActive()) {
            return;
        }
        hSlideTarget = robot.hSlideMotor.getCurrentPosition() + (int) (20);
        robot.hSlideMotor.setTargetPosition(hSlideTarget);
        robot.hSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        robot.hSlideMotor.setPower(0.1);
        while( (runtime.seconds() < timeoutS) &&
                robot.hSlideMotor.getCurrentPosition() < hSlideTarget &&
                (robot.hSlideMotor.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d", hSlideTarget);
            telemetry.addData("Path2", "Running at %7d",
                    robot.hSlideMotor.getCurrentPosition());
            telemetry.update();
        }

        robot.hSlideMotor.setPower(0);
        robot.hSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
    /*
    public void vacuum (double speed, double vacuumInches)
    {
        int vacuumTarget;

        if(opModeIsActive())
        {
            vacuumTarget = robot.vacuum1.getCurrentPosition() + (int) (vacuumInches * COUNTS_PER_INCH);
            robot.vacuum1.setTargetPosition(vacuumTarget);
            // robot.vacuum2.setTargetPosition(vacuumTarget);

            robot.vacuum1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // robot.vacuum2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.vacuum1.setPower(Math.abs(speed));
            // robot.vacuum2.setPower(Math.abs(speed));

            while(opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.vacuum1.isBusy() /* && robot.vacuum2.isBusy()))
            {
                telemetry.addData("Path1", "Running to %7d :%7d", vacuumTarget);
                /*telemetry.addData("Path2", "Running at %7d :%7d",
                robot.vacuum1.getCurrentPosition();
                //robot.vacuum2.getCurrentPosition());
                telemetry.update();
            }

            robot.vacuum1.setPower(0);
            //robot.vacuum2.setPower(0);

            robot.vacuum1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.vacuum2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

    public void elevator (double speed, double elevatorInches) //pulley system
    {
        int elevatorTarget;

        elevatorTarget = robot.elevator.getCurrentPosition() + (int) (elevatorInches * COUNTS_PER_INCH);
        robot.elevator.setTargetPosition(elevatorTarget);

        robot.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        robot.elevator.setPower(Math.abs(speed));

        while(opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.elevator.isBusy()))
        {
            telemetry.addData("Path1", "Running to %7d :%7d", elevatorTarget);
            //telemetry.addData("Path2", "Running at %7d :%7d",
            robot.elevator.getCurrentPosition();
            telemetry.update();
        }

        robot.elevator.setPower(0);

        robot.elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void guide(double speed, double guideInches)
    {
        int guideTarget;

        guideTarget = robot.guide.getCurrentPosition() + (int) (guideInches * COUNTS_PER_INCH);

        robot.guide.setTargetPosition(guideTarget);

        robot.guide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        robot.guide.setPower(Math.abs(speed));

        while(opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.guide.isBusy()))
        {
            telemetry.addData("Path1", "Running to %7d :%7d", guideTarget);
            //telemetry.addData("Path2", "Running at %7d :%7d",
            robot.guide.getCurrentPosition();
            telemetry.update();
        }

        robot.guide.setPower(0);
        robot.guide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }




    public void box(double box_boxpos) //box that contains the loot
    {
        if(opModeIsActive())
        {
            robot.box.setPosition(box_boxpos);
        }
    }

    public void gate(double gate_gatepos)
    {
        if(opModeIsActive())
        {
            robot.gate.setPosition(gate_gatepos);
        }

    }



    public void launch (double speed, double fly_wheelinches, double rampinches){
        int newFlyWheelTarget;
        int newRampTarget;

        if (opModeIsActive()){
            newFlyWheelTarget = robot.fly_wheel.getCurrentPosition() + (int)(fly_wheelinches * COUNTS_PER_INCH);
            newRampTarget = robot.ramp.getCurrentPosition() + (int)(rampinches * COUNTS_PER_INCH);

            robot.fly_wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ramp.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.fly_wheel.setPower(Math.abs(speed));
            robot.ramp.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.vSlideMotor.isBusy()) ){

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newFlyWheelTarget, newRampTarget);
                //telemetry.addData("Path2", "Running at %7d :%7d",
                robot.fly_wheel.getCurrentPosition();
                robot.ramp.getCurrentPosition();
                telemetry.update();
            }

            robot.fly_wheel.setPower(0);
            robot.ramp.setPower(0);

            robot.fly_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.ramp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}
*/