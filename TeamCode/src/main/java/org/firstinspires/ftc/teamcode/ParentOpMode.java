/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


/**
 * Original FTC opmode header block
 *
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 **/

/** Parent OpMode Class
 * All Teleop and Autonomous OpModes should inherit from (extend) ParentOpMode.
 * Each child/subclass OpMode should have its own unique runOpMode() method that will
 * override the ParentOpMode runOpMode() method.
 **/

@TeleOp(name="Parent Opmode", group="Linear Opmode")
@Disabled
public class ParentOpMode extends LinearOpMode {

    // Declare OpMode members, hardware variables
    public ElapsedTime runtime = new ElapsedTime();

    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;

    private DcMotor LiftMotor = null;
    private CRServo IntakeServo = null;
    private Servo PushyServo = null;

    private DigitalChannel StopLiftSwitch = null;

    IMU imu;



    //Other Global Variables
    //put global variables here...
    public double ServoPosition = 0;
    //
    //

    Toggle drive_Toggle = new Toggle(drive_toggle_button());

    public void initialize(){
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Driver Station app or Driver Hub).
        rightFront = hardwareMap.get(DcMotor.class, "rf_drive");
        rightBack = hardwareMap.get(DcMotor.class, "rb_drive");
        leftFront = hardwareMap.get(DcMotor.class, "lf_drive");
        leftBack = hardwareMap.get(DcMotor.class, "lb_drive");

        LiftMotor = hardwareMap.get(DcMotor.class, "lift");

        IntakeServo = hardwareMap.get(CRServo.class, "InT_Servo");

        PushyServo = hardwareMap.get(Servo.class, "push_servo");

        StopLiftSwitch = hardwareMap.get(DigitalChannel.class, "stop_lift_switch");



        //Set motor run mode (if using SPARK Mini motor controllers)


        //Set Motor  and servo Directions
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);

        LiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        IntakeServo.setDirection(CRServo.Direction.FORWARD);

        PushyServo.setDirection(Servo.Direction.FORWARD);


        //Set range for special Servos
        //wobbleLift.scaleRange(0.15,.85); //Savox PWM range is between 0.8 and 2.2 ms. REV Hub puts out 0.5-2.5ms.

        //Set brake or coast modes.
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //BRAKE or FLOAT (Coast)
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Set digital I/O modes
        StopLiftSwitch.setMode(DigitalChannel.Mode.INPUT);

        gyroInitialize();

        //Update Driver Station Status Message after init
        telemetry.addData("Status:", "Initialized");
        telemetry.update();
    }



    /**
     * runOpMode() will be overridden in child OpMode.
     * Basic structure should remain intact (init, wait for start, while(opModeIsActive),
     * Additionally, Emergency conditions should be checked during every cycle
     */
    @Override
    public void runOpMode() {


        initialize();

        // Init loop - optional
        while(opModeInInit()){
            // Code in here will loop continuously until OpMode is started
        }

        // Wait for the game to start (driver presses PLAY) - May not be needed if using an init Loop
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // code here should never actually execute in parent opmode.
            // This function will be be overridden by child opmode classes


          checkEmergencyStop();


            //checkEmergencyStop(); // Stops motors and Terminates if buttons are pressed
            //without additional code in the while(opModeIsActive) loop.

            telemetry.update();
        }
    }

    /*****************************/
    //Controls should be mapped here to avoid accessing gamepad directly in other functions/methods
    //This also makes it simpler to re-map controls as desired
    //CONTROLLER MAP

    // Thumbsticks
    public double left_sticky_x(){
        return gamepad1.left_stick_x;
    }
    public double left_sticky_y(){
        return -gamepad1.left_stick_y;
    }
    public double right_sticky_x(){
        return gamepad1.right_stick_x;
    }
    public double right_sticky_y(){
        return -gamepad1.right_stick_y;
    }


    //Buttons
    public boolean PositionOneButton() { return gamepad1.a; }

    public boolean PositionTwoButton() { return gamepad1.b; }

    public boolean PositionThreeButton() { return gamepad1.y; }

    public boolean Intake_button() { return gamepad1.left_bumper; }
    public boolean Intake_Reverse() {
        if (gamepad1.left_trigger >= .5) {
            return true;
        }
        else {
            return false;
        }
    }
    public boolean Lift_Up() { return gamepad1.right_bumper; }
    public boolean Lift_Down() {
        if (gamepad1.right_trigger >= .5) {
            return true;}
        else{
            return false;
        }
    }



    public boolean drive_toggle_button(){
        return gamepad1.start;
    }

    public boolean Push_Out() { return gamepad1.dpad_up;}
    public boolean Push_Back(){ return gamepad1.dpad_down;}
    public boolean Push_Mid() { return gamepad1.dpad_right;}


    public boolean emergencyButtons(){
        // check for combination of buttons to be pressed before returning true
       if(gamepad1.b == true && gamepad1.y == true){
           return true;
       }
       else{
           return false;
       }


    }
    public boolean yawResetButton(){
        return gamepad1.back;
    }



    public boolean PaperAirplaneButton(){
        return gamepad1.x;
    }

    /*
    public boolean triggerButton(){
        if((gamepad1.right_trigger>.25)||(gamepad2.right_trigger>.25)){
            return true;         // Converts analog triggers into digital button presses (booleans)
        }
        else{
            return false;
        }
    }
    */

    /****************************/
    // Emergency Stop Functions
    // Only one of these is needed.
    // If using boolean version, call to function will need to be
    // placed in conditional (if/then) statement with code to break from loop or terminate opmode.



    public void checkEmergencyStop(){
        if(emergencyButtons()){
            stopDrive();
            terminateOpModeNow();   // Force exit of OpMode
        }
    }



    /*****************************/
    //Drive Methods

    // Assign left and right drive speed using arguments/parameters rather than hardcoding
    // thumb stick values inside function body. This will allow tank drive to be reused for
    // autonomous programs without additional work
    public void tankDrive(double left, double right){
        rightFront.setPower(right);
        rightBack.setPower(right);
        leftFront.setPower(left);
        leftBack.setPower(left);

        telemetry.addData( "left power " , left);
        telemetry.addData("right power " , right);


    }

    public void Field_Centric_drive (){
        double Rotation = -right_sticky_x();

        double DriveAngle = Math.atan2(left_sticky_y(), left_sticky_x()) - Math.toRadians(gyroAngle());
        double magnitude = Math.hypot(left_sticky_x(), left_sticky_y());

        double leftFrontWheel = magnitude*Math.cos(DriveAngle + (Math.PI/4) + (Rotation));
        double rightFrontWheel = magnitude*Math.sin(DriveAngle + (Math.PI/4) - Rotation);
        double leftBackWheel = magnitude*Math.sin(DriveAngle + (Math.PI/4) + (Rotation));
        double rightBackWheel = magnitude*Math.cos(DriveAngle + (Math.PI/4) - Rotation);

        leftFront.setPower(leftFrontWheel);
        leftBack.setPower(leftBackWheel);
        rightFront.setPower(rightFrontWheel);
        rightBack.setPower(rightBackWheel);

    }

    public void Robot_Centric_drive (){
        double Rotation = -right_sticky_x();

        double DriveAngle = Math.atan2(left_sticky_y(), left_sticky_x());
        double magnitude = Math.hypot(left_sticky_x(), left_sticky_y());

        double leftFrontWheel = magnitude*Math.cos(DriveAngle + (Math.PI/4) + (Rotation));
        double rightFrontWheel = magnitude*Math.sin(DriveAngle + (Math.PI/4) - Rotation);
        double leftBackWheel = magnitude*Math.sin(DriveAngle + (Math.PI/4) + (Rotation));
        double rightBackWheel = magnitude*Math.cos(DriveAngle + (Math.PI/4) - Rotation);

        leftFront.setPower(leftFrontWheel);
        leftBack.setPower(leftBackWheel);
        rightFront.setPower(rightFrontWheel);
        rightBack.setPower(rightBackWheel);

    }

    public void driveToggle(){
        boolean fieldCentric = drive_Toggle.Toggle_Button();

        if (fieldCentric) {
            Field_Centric_drive();
        }
        else {
            Robot_Centric_drive();
        }
    }

    public void stopDrive(){
        tankDrive(0,0);
    }


    /*****************************/
    //Lift Methods (Functions)
    public boolean StopLift (){
        return StopLiftSwitch.getState();
    }

    public int GetLiftPosition(){
        telemetry.addData("Lift_position", GetLiftPosition());
        return LiftMotor.getCurrentPosition();
    }


    public void ResetEncoder() {
        if (StopLift() == true) {
            LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void Run_Lift() {
        double liftPower = .75;
        if(Lift_Up() == true) {
            LiftMotor.setPower(liftPower);
        }
        if (Lift_Down() == true && StopLift() == false) {
            LiftMotor.setPower(-liftPower);
        }
        else{
            liftPower = 0;
            LiftMotor.setPower(liftPower);
        }

        telemetry.addData("lift power ", liftPower);

    }

    public void GoPositionOne(){
        if ( PositionOneButton() == true){
            LiftMotor.setTargetPosition(120);
        }
    }


    /*****************************/
    //More Methods (Functions)

    public void RunIntake(){
        double intakePower = .75;
        if(Intake_button() == true) {
            IntakeServo.setPower(intakePower);
        }

        if(Intake_Reverse() == true) {
            IntakeServo.setPower(-intakePower);
        }
        else{ intakePower = 0;
            IntakeServo.setPower(intakePower);
        }

        telemetry.addData("intake power ", intakePower);

    }

    public void PushPush(){
        double OUT = .68;
        double MIDDLE = .57;
        double IN = .33;
        String pushyposition = "?";
        if(Push_Out() == true) {
            pushyposition = "OUT";
            PushyServo.setPosition(OUT);
        }
        if (Push_Back() == true) {
            pushyposition = "IN";
            PushyServo.setPosition(IN);
        }
        if(Push_Mid() == true) {
            pushyposition = "MIDDLE";
            PushyServo.setPosition(MIDDLE);
        }
        telemetry.addData("pushy placement ", pushyposition);
    }


    public void servoTestTest(){
        if (gamepad1.dpad_up){
            ServoPosition += .001;
        }
        if (gamepad1.dpad_down){
            ServoPosition -= .001;
        }
        PushyServo.setPosition(ServoPosition);

        telemetry.addData("Push placement ", ServoPosition);
    }


    /*****************************/
    //Autonomous Functions

    /*****************************/
    //Encoder Functions


    /*****************************/
    //Gyro Functions
    private void gyroInitialize() {

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection LogoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection USBDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot TotalDirection = new RevHubOrientationOnRobot(LogoDirection, USBDirection);
        IMU.Parameters IMUParameters = new IMU.Parameters(TotalDirection);
        imu.initialize(IMUParameters);
    }


    public double gyroAngle() {

        YawPitchRollAngles YEEHAWOrientation = imu.getRobotYawPitchRollAngles();
        double yawHolder3000 = YEEHAWOrientation.getYaw(AngleUnit.DEGREES);
        return yawHolder3000;
    }

    public void gyroReset() {
        imu.resetYaw();
    }

    public void ManualResetGyro(){
        if( yawResetButton() == true) {
            gyroReset();
        }
    }



    //TODO:
    //  Helper Class - Create Gyro Heading Offset Holder Class
    //  Helper Class - Create Toggle Class
    //      - add debounce
    //  Change intake to motors - not using servos
    //  use combination of buttons for pushypushpush thingy for OUT Position to save on buttons and prevent accidental pixel drops?
    //  Incorporate sensor(s) for lift (encoder, limit switch) to allow set heights/positions

    //TODO: AUTONOMOUS
    //  Holonomic Auto Drive function (time-based)
    //  Auto Rotate function (using gyro)

    //TODO: (Maybe, Hopefully) - Advanced
    //  Computer Vision - AprilTags
    //                  - Align with Tag (Strafe)?
    //                  - Rotate toward Tag?
    //  Computer Vision/ML/TensorFlow - Object Detection (Team Prop)
    //                  - See https://teachablemachine.withgoogle.com/train/image
}