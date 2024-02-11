/* Copyright (c) 2019 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "REDAprilTagCENTERPixelPlacement", group = "Concept")
//@Disabled
public class VisionPixelPlacementOpRedModeFromBack extends VisionParentOopMode {


    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "CenterStage.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
       "Pixel",
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override

    public void runOpMode() {
    initialize();
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                //.setDrawAxes(true)
                //.setDrawCubeProjection(true)
                //.setDrawTagID(true)
                //.setDrawTagOutline(true)
                .setLensIntrinsics(622.001f, 622.001f,319.803f, 241.251f)
                .build();


      initDoubleVision();


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryTfod();
                telemetryAprilTag();

                telemetry.addData("Current_Gyro", gyroAngle());
                // Push telemetry to the Driver Station.
                telemetry.update();


                Auto_Robot_Centric_Drive_Time(.5,270,0, 1320);
                sleep(100);
                gyroRotationAngle(.2,90);
                sleep(400);
                for(int i = 0; i < 100; i++){
                    AprilTagDrivingAlignment(8,0,.3);

                }
                //End Helen's Code...

                //start Finn code...
                Auto_Robot_Centric_Drive_Time(.5,270,0, 1320);
                gyroRotationAngle(.2,180);


                // Share the CPU.
                sleep(200000);
                break;
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()


}   // end class


//XAprilTagPositionInReferenceToTheApproximentPositionOfPercyThePrimaryNearPerfectProtecterOfPreciousAndVeryPrettyParticularPurplePixelsAlthoughWeQuestionHisLIfeDescisionsWeSupportOurFriendAsWeShouldBecausePercyIsThePrimaryRobotFriendOfProgrammersAndAllOthersThatHaveProvidedAddionalAssistenceToPercysUpbringingAsToNotBeOnTheHitListOfWhichPercyMayPossiblyPartakeInHoweverPercysHobbiesAreNotExactlyAParticularPartOfThisPrototypeConversationOfWhichExistsAsPartOfAFunctionAlthoughThisIsNotAFunctionAndInsteadAsAVariable