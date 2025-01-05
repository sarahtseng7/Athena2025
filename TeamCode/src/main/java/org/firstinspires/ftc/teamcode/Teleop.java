/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the Gamepad1 left stick moves the robot FWD and back, the Gamepad1 Right stick turns left and right.
 * Gamepad2 left stick will control the ball collector motors to suck in
 * the ball or shoot them. Gamepad2 left stick will also control the horizontal roller to move the balls in.
 * or out.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out thae @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop", group="Athenabot")
//@Disabled
public class Teleop extends LinearOpMode {



    /* Declare OpMode members. */

    Hardware robot  = new Hardware();

    // Use a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    //grab must go before stack a and b
    //out x and y

    //final double PUSH_SPEED = 0.05;
    //final double PUSH2_SPEED = 0.05;
    final double MAX_SPEED = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            //holonomic(Speed, Turn, Strafe, MAX_SPEED );

            //Left Front = +Speed + Turn - Strafe      Right Front = +Speed - Turn + Strafe
            //Left Rear  = +Speed + Turn + Strafe      Right Rear  = +Speed - Turn - Strafe

            //double Speed = -gamepad1.left_stick_y;
            //double Turn = gamepad1.left_stick_x;
            double Turn = gamepad1.left_stick_x;
            double Strafe = gamepad1.right_stick_x;
            double Speed = gamepad1.left_stick_y;
            //robot.turn.setPosition(gamepad2.left_stick_x);
            //robot.push2.setPosition(-0.2);
            //telemetry.addData("after" , "opmode");
            //telemetry.update();

            //wheels code
            double Magnitude = Math.abs(Speed) + Math.abs(Turn) + Math.abs(Strafe);
            Magnitude = (Magnitude > 1) ? Magnitude : 1; //Set scaling to keep -1,+1 rangestraf
            // Wheels on joystick - gamepad 1

            if (robot.left_front_drv_Motor != null) {
                robot.left_front_drv_Motor.setPower(
                        Range.scale((robot.scaleInput(-Speed) +
                                        robot.scaleInput(Turn) -
                                        robot.scaleInput(-Strafe)),
                                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
            }
            if (robot.left_back_drv_Motor != null) {
                robot.left_back_drv_Motor.setPower(
                        Range.scale((robot.scaleInput(Speed) -
                                        robot.scaleInput(Turn) +
                                        robot.scaleInput(Strafe)),
                                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
            }
            if (robot.right_front_drv_Motor != null) {
                robot.right_front_drv_Motor.setPower(
                        Range.scale((robot.scaleInput(Speed) +
                                        robot.scaleInput(Turn) -
                                        robot.scaleInput(-Strafe)),
                                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
            }
            if (robot.right_back_drv_Motor != null) {
                robot.right_back_drv_Motor.setPower(
                        Range.scale((robot.scaleInput(Speed) +
                                        robot.scaleInput(Turn) -
                                        robot.scaleInput(Strafe)),
                                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
            }

            //viper slide code
            if(robot.vSlideMotor != null){
                if(gamepad2.dpad_up) {
                    robot.vSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    robot.vSlideMotor.setPower(0.4);
                } else if (gamepad2.dpad_down) {
                    robot.vSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    robot.vSlideMotor.setPower(0.4);
                } else {
                    robot.vSlideMotor.setPower(0.0);
                }
            }

            if(robot.hSlideMotor != null){
                if(gamepad1.dpad_up) {
                    robot.hSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    robot.hSlideMotor.setPower(0.3);
                } else if (gamepad1.dpad_down) {
                    robot.hSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                    robot.hSlideMotor.setPower(0.3);
                } else {
                    robot.hSlideMotor.setPower(0.0);
                }
            }

            if(robot.vSlideMotor != null){
                if (gamepad2.right_trigger > 0) {
                    resetRuntime();
                    while (getRuntime() < 8.55) {
                        while (getRuntime() < 4.1) {
                            while (getRuntime() < 3.2) {
                                robot.vSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                                robot.vSlideMotor.setPower(0.4);
                            }
                            robot.basket.setPosition(0.96);
                            sleep(35);
                            robot.basket.setPosition(0.23);
                        }
                        robot.vSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                        robot.vSlideMotor.setPower(0.4);
                    }
                }
            }
            //servos code
            if (robot.basket != null) {
                if (gamepad1.y) {
                    robot.basket.setPosition(1.00);
                    Thread.sleep(1000);
                    robot.basket.setPosition(0.22);
                } else if (gamepad1.a) {
                    robot.basket.setPosition(0.22);
                }
            }


            if (gamepad2.x) {
                robot.claw2.setPosition(0.45);
            }
            if (gamepad2.b) {
                robot.claw2.setPosition(0.45);
            }

            if (opModeIsActive()) {
                if (gamepad1.x ) {
                    resetRuntime();
                    while (getRuntime() < 15) {
                        robot.vSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                        robot.vSlideMotor.setPower(0.15);
                    }
                }
            }

            if (gamepad2.y) {
                robot.claw2.setPosition(0.33); //close claw
                Thread.sleep(100);
                robot.claw2.setPosition(0.45); //open claw
                Thread.sleep(100);
                robot.arm.setPosition(1.0);

            } else if (gamepad2.a) {
                robot.claw2.setPosition(0.33); //close claw
                Thread.sleep(350);
                robot.arm.setPosition(0.3);
            }

/*
            if(robot.claw1 != null){
                if(gamepad2.dpad_left) {
                    robot.claw1.setDirection(CRServo.Direction.FORWARD);
                    robot.claw1.setPower(0.50);
                } else if (gamepad2.dpad_right) {
                    robot.claw1.setDirection(CRServo.Direction.REVERSE);
                    robot.claw1.setPower(0.50);
                } else {
                    robot.claw1.setPower(0.0);
                }
            }

            if(robot.lid1 != null) {
                if (gamepad2.x) {
                    robot.lid1.setPosition(1.0);
                } else if (gamepad2.b) {
                    robot.lid1.setPosition(0.0);
                }
            }
*/

//claw1 original code before turned into crservo
            /*if(robot.claw1 != null) {
                if(gamepad2.x){
                    robot.claw1.setPosition(0.00);
                } else if (gamepad2.b) {
                    robot.claw1.setPosition(-0.35);
                }
            }
*/



/*
            if(robot.elevator != null && robot.elevator2 != null) {
                if(gamepad2.dpad_down) {
                    robot.elevator.setDirection(DcMotorSimple.Direction.FORWARD);
                    robot.elevator.setPower(robot.elevator_power);
                    robot.elevator2.setDirection(DcMotorSimple.Direction.FORWARD);
                    robot.elevator2.setPower(robot.elevator2_power);
                } else if (gamepad2.dpad_up) {
                    robot.elevator.setDirection(DcMotorSimple.Direction.REVERSE);
                    robot.elevator.setPower(robot.elevator_power);
                    robot.elevator2.setDirection(DcMotorSimple.Direction.REVERSE);
                    robot.elevator2.setPower(robot.elevator2_power);

                } else {
                    robot.elevator.setPower(0.0);
                    robot.elevator2.setPower(0.0);
                }
            }
            if(robot.guide != null) {
                robot.guide.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.guide.setPower(robot.guide_power);
            }
        */
        }
    }
}






