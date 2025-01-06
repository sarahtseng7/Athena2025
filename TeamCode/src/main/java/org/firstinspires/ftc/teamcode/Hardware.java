package org.firstinspires.ftc.teamcode;

//import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "roller_drive"
 * Servo channel:  Servo to open left claw_left:  "left_hand"
 * Servo channel:  Servo to open right claw_left: "right_hand"
 */
public class Hardware
{
    /* Public OpMode members. */


    public DcMotor  left_front_drv_Motor   = null;    // Robot left front drive
    public DcMotor  right_front_drv_Motor  = null;    // Robot right front drive
    public DcMotor  left_back_drv_Motor   = null;    // Robot left back drive
    public DcMotor  right_back_drv_Motor  = null;    // Robot right back drive
    //public DcMotor  hang_motor = null; // Robot hang and landing

    //public DcMotor  hinge = null; // hinge for arm
    //public DcMotor  vSlideMotor = null ; //intake motor for arm
    public DcMotor vSlideMotor = null; // intake motor for clamp
    public DcMotor hSlideMotor = null;

    public DcMotor fly_wheel = null;
    public DcMotor ramp = null;
    public DcMotor vacuum1 = null;
    public DcMotor guide = null;
    public DcMotor elevator = null;

    public DcMotor slide1 = null;

    public DcMotor slide2 = null;

    public DcMotor elevator2 = null;


    //public DcMotor hSlideMotor = null;
    //public DcMotor flip_motor = null; // slide motor for stacking
    // public DcMotor stack2_motor = null;


    public CRServo claw1 = null;
    public Servo claw2 = null;
    public Servo basket = null;
    public Servo arm = null;

    public Servo lid1 = null;

    public ColorSensor colorSensor = null; // Sensor for Beacon
    public LightSensor LightSensorBottom = null; // Sensor for line following
    //public OpticalDistanceSensor odsSensor = null;  // Sensor for dist measurement



    //Use MR Core Device Discovery to change address
    //I2cAddr i2CAddressColorFront = I2cAddr.create8bit(0x3c);
    //I2cAddr i2CAddressColorBottom = I2cAddr.create8bit(0x4c);


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // left frond drive motor
        try {

            left_front_drv_Motor = hwMap.dcMotor.get("frontLeft");
        } catch (Exception p_exeception) {


            left_front_drv_Motor = null;
        }
        //left back drive motor

        try {

            left_back_drv_Motor = hwMap.dcMotor.get("backLeft");
        } catch (Exception p_exeception) {


            left_back_drv_Motor = null;
        }
        // right front drive motor
        try {

            right_front_drv_Motor = hwMap.dcMotor.get("frontRight");
        } catch (Exception p_exeception) {

            right_front_drv_Motor = null;
        }
        //right back drive motor
        try {

            right_back_drv_Motor = hwMap.dcMotor.get("backRight");
        } catch (Exception p_exeception) {


            right_back_drv_Motor = null;
        }

        try {
            colorSensor = hwMap.colorSensor.get("color");

        } catch (Exception p_exception) {

            colorSensor = null;
        }
        //hinge motor for arm

        //try
        //{

      /*    hinge   = hwMap.dcMotor.get("hinge");
      }
      catch (Exception p_exeception)
      {


          hinge = null;
      }*/
        //stack motor for stacking skystones
      /*try {

          flip_motor = hwMap.dcMotor.get("extension");
      } catch (Exception p_exeception) {


          flip_motor = null;
      }*/

      /*try {

          stack2_motor = hwMap.dcMotor.get("stack2");
      } catch (Exception p_exeception) {


          stack2_motor = null;
      }*/


        //intake motor for clamp
        try {

            vSlideMotor = hwMap.dcMotor.get("VSlide");
        } catch (Exception p_exception) {


            vSlideMotor = null;
        }
        try {

            hSlideMotor = hwMap.dcMotor.get("HSlide");
        } catch (Exception p_exception){

            hSlideMotor = null;
        }
        try {
            fly_wheel = hwMap.dcMotor.get("fly_wheel");
        } catch (Exception p_exception){
            fly_wheel = null;
        }

        try {
            ramp = hwMap.dcMotor.get("ramp");
        } catch (Exception p_exception){
            ramp = null;
        }

        try {
            vacuum1 = hwMap.dcMotor.get("vacuum1");
        } catch (Exception p_exception){
            vacuum1 = null;
        }

        try {
            guide = hwMap.dcMotor.get("guide");
        } catch (Exception p_exception){
            guide = null;
        }

        try{
            elevator = hwMap.dcMotor.get("lever");
        } catch (Exception p_exception){
            elevator = null;
        }

        try{
            slide1 = hwMap.dcMotor.get("slide1");
        } catch (Exception p_exception){
            slide1 = null;
        }

        try{
            slide2 = hwMap.dcMotor.get("slide2");
        } catch (Exception p_exception){
            slide2 = null;
        }
      /*try {

          hSlideMotor = hwMap.dcMotor.get("intake2");
      } catch (Exception p_exeception) {


          hSlideMotor = null;
      }*/

        // Servos :




        try {

        } catch (Exception p_exeception) {
            claw1 = null;
        }

        /*try {
            //v_motor_left_drive = hardwareMap.dcMotor.get ("l_drv");
            //v_motor_left_drive.setDirection (DcMotor.Direction.REVERSE);
            claw2 = hwMap.servo.get("claw2");
            if (claw2 != null) {
                claw2.setPosition(SERVO_CLAW2_MIN);
            }


        }
        catch (Exception p_exeception) {
            //m_warning_message ("l_drv");
            //   DbgLog.msg (p_exeception.getLocalizedMessage ());

            claw2 = null;
        }*/
        try {
            //v_motor_left_drive = hardwareMap.dcMotor.get ("l_drv");
            //v_motor_left_drive.setDirection (DcMotor.Direction.REVERSE);
            lid1 = hwMap.servo.get("lid1");



        }
        catch (Exception p_exeception) {
            //m_warning_message ("l_drv");
            //   DbgLog.msg (p_exeception.getLocalizedMessage ());

            lid1 = null;
        }


        try {
            basket = hwMap.servo.get("basket");
            if (basket != null) {
                basket.setPosition(0.23);
            }
        }
        catch (Exception p_exeception) {
            //m_warning_message ("l_drv");


            basket = null;
        }

        try {
            //v_motor_left_drive = hardwareMap.dcMotor.get ("l_drv");
            //v_motor_left_drive.setDirection (DcMotor.Direction.REVERSE);
            arm = hwMap.servo.get("arm");
            if (arm != null) {
                arm.setPosition(0.7);
            }


        }
        catch (Exception p_exeception) {
            //m_warning_message ("l_drv");
            //   DbgLog.msg (p_exeception.getLocalizedMessage ());

            arm = null;
        }

        try {
            //v_motor_left_drive = hardwareMap.dcMotor.get ("l_drv");
            //v_motor_left_drive.setDirection (DcMotor.Direction.REVERSE);
            claw2 = hwMap.servo.get("claw2");
            if (claw2 != null) {
                claw2.setPosition(0.33);
            }


        }
        catch (Exception p_exeception) {
            //m_warning_message ("l_drv");
            //   DbgLog.msg (p_exeception.getLocalizedMessage ());

            claw2 = null;
        }

        //set dc motors to run without an encoder  and set intial power to 0
        //l_f_drv
        if (left_front_drv_Motor != null) {
            //l_return = left_drv_Motor.getPower ();
            left_front_drv_Motor.setDirection(DcMotor.Direction.FORWARD); // FORWARD was moving it backwards
            left_front_drv_Motor.setPower(0);
            left_front_drv_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        //l_b_drv
        if (left_back_drv_Motor != null) {
            //l_return = left_drv_Motor.getPower ();
            left_back_drv_Motor.setDirection(DcMotor.Direction.REVERSE); // FORWARD was moving it backwards
            left_back_drv_Motor.setPower(0);
            left_back_drv_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        //r_f_drv
        if (right_front_drv_Motor != null) {
            //l_return = left_drv_Motor.getPower ();
            right_front_drv_Motor.setDirection(DcMotor.Direction.FORWARD);// REVERSE was moving it backwards
            right_front_drv_Motor.setPower(0);
            right_front_drv_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        //r_b_drv
        if (right_back_drv_Motor != null) {
            //l_return = left_drv_Motor.getPower ();
            right_back_drv_Motor.setDirection(DcMotor.Direction.FORWARD);// REVERSE was moving it backwards
            right_back_drv_Motor.setPower(0);
            right_back_drv_Motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        if (claw1 != null) {
            //l_return = left_drv_Motor.getPower ();
            claw1.setDirection(CRServo.Direction.FORWARD);// REVERSE was moving it backwards
            claw1.setPower(0);
        }

        //intake
        if (vSlideMotor != null) {
            vSlideMotor.setDirection(DcMotor.Direction.FORWARD);
            vSlideMotor.setPower(0);
            vSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            vSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (hSlideMotor != null){
            hSlideMotor.setDirection(DcMotor.Direction.FORWARD);
            hSlideMotor.setPower(0);
            hSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (fly_wheel != null) {
            fly_wheel.setDirection(DcMotor.Direction.FORWARD);
            fly_wheel.setPower(0);
            fly_wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (guide != null){
            guide.setDirection(DcMotor.Direction.FORWARD);
            guide.setPower(0);
            guide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (elevator != null){
            elevator.setDirection(DcMotor.Direction.FORWARD);
            elevator.setPower(0);
            elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (slide1 != null){
            slide1.setDirection(DcMotor.Direction.FORWARD);
            slide1.setPower(0);
            slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (slide2 != null){
            slide2.setDirection(DcMotor.Direction.FORWARD);
            slide2.setPower(0);
            slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

          /*if (hSlideMotor != null) {
              hSlideMotor.setDirection(DcMotor.Direction.REVERSE);
              hSlideMotor.setPower(0);
              hSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
          }*/

      /*
      initalize the colorSensor and colorSensor
      */
        try {
            colorSensor = hwMap.colorSensor.get("color");
        } catch (Exception p_exeception) {
            //m_warning_message ("colr_f");
            //   DbgLog.msg (p_exeception.getLocalizedMessage ());
            colorSensor = null;
        }


        try {
            LightSensorBottom = hwMap.lightSensor.get("ods");
        } catch (Exception p_exeception) {
            //m_warning_message ("ods");
            //   DbgLog.msg (p_exeception.getLocalizedMessage ());
            LightSensorBottom = null;
        }


        if (colorSensor != null) {
            //ColorFront reads beacon light and is in passive mode
            //colorSensor.setI2cAddress(i2CAddressColorFront);
            colorSensor.enableLed(false);
        }

        if (LightSensorBottom != null) {
            //OpticalDistance sensor measures dist from the beacon
            LightSensorBottom.enableLed(false);
        }

    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear. This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    public double scaleInput( double dVal ) {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }


    /**
     * Access the left front drive motor's power level.
     */
    public double a_left_front_drive_power()
    {
        double l_return = 0.0;
        if (left_front_drv_Motor != null)
        {
            l_return = left_front_drv_Motor.getPower ();
        }

        return l_return;

    } // a_left_drive_power

    /**
     * Access the left back drive motor's power level.
     */
    double a_left_back_drive_power()
    {
        double l_return = 0.0;
        if (left_back_drv_Motor != null)
        {
            l_return = left_back_drv_Motor.getPower ();
        }

        return l_return;

    } // a_left_drive_power


    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
