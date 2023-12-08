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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
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

    private DcMotor LiftMotorRight = null;

    private DcMotor LiftMotorLeft = null;

    private DcMotorEx IntakeMotor = null;
    private Servo PushyServo = null;

    private DigitalChannel LiftIsBottomNowOkSwitch = null;

    private Servo WristLeft = null;
    private Servo WristRight = null;
    IMU imu;

    private Servo AirplaneLauncher = null;



    //Other Global Variables
    //put global variables here...
    public double ServoPosition = 0;

    public int LiftPosition;

    //Lift Positions
    int Bottom = 0000;
    int MiniMinimum = 20;
    int MinimumToMoveWrist = 50;
    int Low = 100;
    int Middle = 200;
    int High = 300;
    int NOSTOPITURBREAKINGMEAAA= 400;

    //Wrist Positions
    double ScorePOS = 0;
    double HomePOS = .4;
    double ClimbPOS = .75;

    //Toggle drive_Toggle = new Toggle(drive_toggle_button());

    public void initialize(){
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Driver Station app or Driver Hub).
        rightFront = hardwareMap.get(DcMotor.class, "rf_drive");
        rightBack = hardwareMap.get(DcMotor.class, "rb_drive");
        leftFront = hardwareMap.get(DcMotor.class, "lf_drive");
        leftBack = hardwareMap.get(DcMotor.class, "lb_drive");

        LiftMotorLeft = hardwareMap.get(DcMotor.class, "lift_left");
        LiftMotorRight = hardwareMap.get(DcMotor.class, "lift_right");

        IntakeMotor = hardwareMap.get(DcMotorEx.class, "int_motor");

        PushyServo = hardwareMap.get(Servo.class, "push_servo");

        LiftIsBottomNowOkSwitch = hardwareMap.get(DigitalChannel.class, "stop_lift_switch");

        WristLeft = hardwareMap.get(Servo.class, "wrist_left");
        WristRight = hardwareMap.get(Servo.class, "wrist_right");

        AirplaneLauncher = hardwareMap.get(Servo.class, "airplane_launcher");


        //Set motor run mode (if using SPARK Mini motor controllers)


        //Set Motor  and servo Directions
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);

        LiftMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        LiftMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        IntakeMotor.setDirection(DcMotor.Direction.REVERSE);

        PushyServo.setDirection(Servo.Direction.FORWARD);

        AirplaneLauncher.setDirection(Servo.Direction.FORWARD);

        WristLeft.setDirection(Servo.Direction.FORWARD);
        WristRight.setDirection(Servo.Direction.FORWARD );


        //Set range for special Servos
        //wobbleLift.scaleRange(0.15,.85); //Savox PWM range is between 0.8 and 2.2 ms. REV Hub puts out 0.5-2.5ms.

        //Set brake or coast modes.
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //BRAKE or FLOAT (Coast)
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LiftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set digital I/O modes
        LiftIsBottomNowOkSwitch.setMode(DigitalChannel.Mode.INPUT);

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
    public boolean Intake_Reverse_Button() {
    if (gamepad1.left_trigger >= .5) {
            return true;
        }
        else {
            return false;
        }
    }

    public boolean Lift_Up_Button() { return gamepad1.right_bumper; }

    public boolean Lift_Down_Button(
    ) {
        if (gamepad1.right_trigger >= .5) {
            return true;}
        else{
            return false;
        }
    }

    public boolean drive_toggle_button(){
        return gamepad1.start;
    }

    public boolean Push_Out_Button() { return gamepad1.dpad_up;}
    public boolean Push_Back_Button(){ return gamepad1.dpad_down;}
    public boolean Push_Mid_Button() { return gamepad1.dpad_right;}

    public boolean WristScoreButton(){return gamepad2.x;}
    public boolean WristRestButton(){return gamepad2.a;}
    public boolean WristClimbButton(){return gamepad2.b;}

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

    public boolean AirplaneButton(){
        return gamepad1.x;}

    /****************************/
    // Emergency Stop
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
        double Rotation = right_sticky_x();

        double DriveAngle = Math.atan2(left_sticky_y(), left_sticky_x()) - Math.toRadians(gyroAngle());
        double magnitude = Math.hypot(left_sticky_x(), left_sticky_y());

        double leftFrontWheel = magnitude*Math.cos(DriveAngle + (Math.PI/4) + Rotation);
        double rightFrontWheel = magnitude*Math.sin(DriveAngle + (Math.PI/4) - Rotation);
        double leftBackWheel = magnitude*Math.sin(DriveAngle + (Math.PI/4) + Rotation);
        double rightBackWheel = magnitude*Math.cos(DriveAngle + (Math.PI/4) - Rotation);

        leftFront.setPower(leftFrontWheel);
        leftBack.setPower(leftBackWheel);
        rightFront.setPower(rightFrontWheel);
        rightBack.setPower(rightBackWheel);
    }
    public void Auto_Field_Centric_drive (double Magnitude, double driveAngle, double rotation){
        double Rotation = rotation;

        double DriveAngle = driveAngle - Math.toRadians(gyroAngle());
        double magnitude = Magnitude;

        double leftFrontWheel = magnitude*Math.cos(DriveAngle + (Math.PI/4)) + Rotation;
        double rightFrontWheel = magnitude*Math.sin(DriveAngle + (Math.PI/4)) - Rotation;
        double leftBackWheel = magnitude*Math.sin(DriveAngle + (Math.PI/4)) + Rotation;
        double rightBackWheel = magnitude*Math.cos(DriveAngle + (Math.PI/4)) - Rotation;

        leftFront.setPower(leftFrontWheel);
        leftBack.setPower(leftBackWheel);
        rightFront.setPower(rightFrontWheel);
        rightBack.setPower(rightBackWheel);
    }
    public void Auto_Field_Centric_drive_time (double Magnitude, double driveAngle, double rotation, int timeMS){
        Auto_Field_Centric_drive(Magnitude, driveAngle, rotation);
        sleep(timeMS);
        stopDrive();
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
/*
    public void driveToggle(){
        boolean fieldCentric = drive_Toggle.Toggle_Button();

        if (fieldCentric) {
            Field_Centric_drive();
        }
        else {
            Robot_Centric_drive();
        }
    }
*/

    public void stopDrive(){
        tankDrive(0,0);
    }


    /*****************************/
    //Lift Methods (Functions)
    public boolean BottomLiftSwitch(){
        return LiftIsBottomNowOkSwitch.getState();
    }

    public int GetLiftPosition(){
        int liftPosiTION;
        liftPosiTION = LiftMotorRight.getCurrentPosition();
        telemetry.addData("Lift_position", liftPosiTION);
        return liftPosiTION;
    }

    public void ResetEncoders() {
            LiftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LiftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void Run_Lift() {
        double liftPower = .75;
        int ABit = 20;
        if (Lift_Up_Button() == true) {
          LiftPosition = LiftPosition + ABit;
        }
        if (Lift_Down_Button() == true) {
           LiftPosition = LiftPosition - ABit;
        }
        if (LiftPosition > NOSTOPITURBREAKINGMEAAA){
            LiftPosition = NOSTOPITURBREAKINGMEAAA;
        }
        if (LiftPosition < MiniMinimum) {
            LiftPosition = MiniMinimum;
        }
        GoPosition(LiftPosition);
        if (BottomLiftSwitch() == true){
            ResetEncoders();

        }

        telemetry.addData("lift power ", liftPower);

    }

    public void HomingLift(){
        LiftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!BottomLiftSwitch() && opModeIsActive()){
            LiftMotorLeft.setPower(.3);
            LiftMotorRight.setPower(.3);
        }

        LiftMotorLeft.setPower(0);
        LiftMotorRight.setPower(0);

        ResetEncoders();
        LiftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void GoPosition(int LiftSpecificPlaceYouAreGoingHereNow){
        double LiftSpeed = .35;

        LiftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LiftMotorLeft.setTargetPosition(LiftSpecificPlaceYouAreGoingHereNow);
        LiftMotorRight.setTargetPosition(LiftSpecificPlaceYouAreGoingHereNow);

        LiftMotorLeft.setPower(LiftSpeed);
        LiftMotorRight.setPower(LiftSpeed);
    }

    public void WristPOS(){
        if (GetLiftPosition() >= MinimumToMoveWrist){
            if (WristScoreButton() == true){
            WristRight.setPosition(ScorePOS);
            WristLeft.setPosition(ScorePOS);
            }
            if (WristRestButton() == true){
                WristRight.setPosition(HomePOS);
                WristLeft.setPosition(HomePOS);
            }
            if (WristClimbButton() == true){
                WristLeft.setPosition(ClimbPOS);
                WristRight.setPosition(ClimbPOS);
            }
        }

        if (GetLiftPosition() < MinimumToMoveWrist){
            GoPosition(MinimumToMoveWrist + 25);
        }
    }

    //TODO make auto wrist command
    /*
    public void AutoWristPOS(){
                WristRight.setPosition();
                WristLeft.setPosition();

    }
*/
    /*****************************/
    //More Methods (Functions)

    public void RunIntake(){
        double intakePower = .75;
        double currentLimit = 2.5;  //may need to increase and/or account for momentary current spikes

        if(Intake_button() == true) {
            IntakeMotor.setPower(intakePower);
        }
        else{
            if(Intake_Reverse_Button() == true) {
                IntakeMotor.setPower(-intakePower);

            }
            else{ intakePower = 0;
                IntakeMotor.setPower(intakePower);
            }
        }

        /*  //move this into Intake_Button() section of if-statement. Probably need to add a tiny sleep (300ms?)
        if(IntakeMotor.getCurrent(CurrentUnit.AMPS) > currentLimit){
            IntakeMotor.setPower(-intakePower);
            telemetry.addData(" CurrentTooHigh, Going Reverse", IntakeMotor.getCurrent(CurrentUnit.AMPS));
        }
        */


        telemetry.addData("intake power ", intakePower);
        telemetry.addData("DcMotor Intake Current", IntakeMotor.getCurrent(CurrentUnit.AMPS));


    }

    public void PushPush(){
        double OUT = .68;
        double MIDDLE = .57;
        double IN = .33;
        String pushyposition = "?";
        if(Push_Out_Button() == true) {
            pushyposition = "OUT";
            PushyServo.setPosition(OUT);
        }
        if (Push_Back_Button() == true) {
            pushyposition = "IN";
            PushyServo.setPosition(IN);
        }
        if(Push_Mid_Button() == true) {
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

    public void airplanePewPew(){
        double firePosition = 1;
        double cockedPosition = 0;

        if (AirplaneButton() == true){
            AirplaneLauncher.setPosition(firePosition);
        }
        else{
            AirplaneLauncher.setPosition(cockedPosition);
        }
    }


    /*****************************/
    //Autonomous Functions

    /*****************************/
    //Encoder Functions


    /*****************************/
    //Gyro Functions
    private void gyroInitialize() {

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection LogoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection USBDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

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