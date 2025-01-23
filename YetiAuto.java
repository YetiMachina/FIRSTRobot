/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * This OpMode is an example driver-controlled (TeleOp) mode for the goBILDA 2024-2025 FTC
 * Into The Deep Starter Robot
 * The code is structured as a LinearOpMode
 *
 * This robot has a two-motor differential-steered (sometimes called tank or skid steer) drivetrain.
 * With a left and right drive motor.
 * The drive on this robot is controlled in an "Arcade" style, with the left stick Y axis
 * controlling the forward movement and the right stick X axis controlling rotation.
 * This allows easy transition to a standard "First Person" control of a
 * mecanum or omnidirectional chassis.
 *
 * The drive wheels are 96mm diameter traction (Rhino) or omni wheels.
 * They are driven by 2x 5203-2402-0019 312RPM Yellow Jacket Planetary Gearmotors.
 *
 * This robot's main scoring mechanism includes an arm powered by a motor, a "wrist" driven
 * by a servo, and an intake driven by a continuous rotation servo.
 *
 * The arm is powered by a 5203-2402-0051 (50.9:1 Yellow Jacket Planetary Gearmotor) with an
 * external 5:1 reduction. This creates a total ~254.47:1 reduction.
 * This OpMode uses the motor's encoder and the RunToPosition method to drive the arm to
 * specific setpoints. These are defined as a number of degrees of rotation away from the arm's
 * starting position.
 *
 * Make super sure that the arm is reset into the robot, and the wrist is folded in before
 * you run start the OpMode. The motor's encoder is "relative" and will move the number of degrees
 * you request it to based on the starting position. So if it starts too high, all the motor
 * setpoints will be wrong.
 *
 * The wrist is powered by a goBILDA Torque Servo (2000-0025-0002).
 *
 * The intake wheels are powered by a goBILDA Speed Servo (2000-0025-0003) in Continuous Rotation mode.
 */


@TeleOp(name="Autonomous", group="Robot")
//@Disabled
public class YetiAuto extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotorSimple  leftBackDrive   = null; //the left back drivetrain motor
    public DcMotor  rightBackDrive  = null; //the right back drivetrain motor
    public DcMotor  leftFrontDrive  = null; //the left front drivetrain motor
    public DcMotor rightFrontDrive = null; //the right front drivetrain motor, controller in servo port
    public DcMotor  armMotor    = null; //the arm motor
    public Servo  intake      = null; //the active intake servo
    public Servo    wrist       = null;
    public CRServo theBitThatExtends = null;

    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
    To find this, we first need to consider the total gear reduction powering our arm.
    First, we have an external 20t:100t (5:1) reduction created by two spur gears.
    But we also have an internal gear reduction in our motor.
    The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
    reduction of ~50.9:1. (more precisely it is 250047/4913:1)
    We can multiply these two ratios together to get our final reduction of ~254.47:1.
    The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
    counts per rotation of the arm. We divide that by 360 to get the counts per degree. */
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation


    /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
    160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it. */

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 0;
    final double ARM_CLEAR_BARRIER         = -20 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = -66 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = -105 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = -136 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = -15  * ARM_TICKS_PER_DEGREE;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_CLAMP   =  1.0;
    final double INTAKE_OPEN    =  0.85;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN   = 1.0;
    final double WRIST_FOLDED_OUT  = 0.7175;

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 17 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;


    @Override
    public void runOpMode() {
        /*
        These variables are private to the OpMode, and are used to control the drivetrain.
         */
        double x;
        double y;
        double xr;
        double maxFront;
        double maxBack;
        
        ElapsedTime runtime;

        /* Define and Initialize Motors */
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFDrive"); //the left drivetrain motor
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFDrive"); //the right SPARKmini motor
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBDrive");
        leftBackDrive = hardwareMap.get(DcMotorSimple.class, "leftBDrive");
        armMotor   = hardwareMap.get(DcMotor.class, "armMotor"); //the arm motor

        runtime = new ElapsedTime();

        /* Most skid-steer/differential drive robots require reversing one motor to drive forward.
        for this robot, we reverse the right motor.*/
        //leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        //rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        
        


        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5,CurrentUnit.AMPS);


        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        armMotor.setTargetPosition((int)(-20 * ARM_TICKS_PER_DEGREE));
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        /* Define and initialize servos.*/
        intake = hardwareMap.get(Servo.class, "intake");
        theBitThatExtends = hardwareMap.get(CRServo.class, "extendo");
        wrist  = hardwareMap.get(Servo.class, "wrist");

        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPosition(INTAKE_CLAMP);
        theBitThatExtends.setPower(0.0);
        
        wrist.setPosition(WRIST_FOLDED_IN);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        /* Wait for the game driver to press play */
        waitForStart();
        
        armMotor.setTargetPosition((int)ARM_SCORE_SPECIMEN);
        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setPosition(WRIST_FOLDED_OUT);

        while (opModeIsActive() && runtime.seconds() < 25) {
            armMotor.setTargetPosition((int)ARM_SCORE_SPECIMEN);
            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wrist.setPosition(WRIST_FOLDED_OUT);
            while (runtime.seconds() <= 3 && runtime.seconds() > 2){
                leftFrontDrive.setPower(0.4);
                rightFrontDrive.setPower(-0.4);
                leftBackDrive.setPower(0.4);
                rightBackDrive.setPower(-0.4);
            }
            leftFrontDrive.setPower(0.0);
            rightFrontDrive.setPower(-0.0);
            leftBackDrive.setPower(0.0);
            rightBackDrive.setPower(-0.0);
            while (runtime.seconds() <= 5 && runtime.seconds() > 3) {
                armMotor.setTargetPosition((int) (ARM_SCORE_SPECIMEN + FUDGE_FACTOR));
                ((DcMotorEx) armMotor).setVelocity(2100);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            while (runtime.seconds() <= 7 && runtime.seconds() > 5) {
                intake.setPosition(INTAKE_OPEN);
            }
            while (runtime.seconds() <= 8 && runtime.seconds() > 7) {
                leftFrontDrive.setPower(-0.4);
                rightFrontDrive.setPower(0.4);
                leftBackDrive.setPower(-0.4);
                rightBackDrive.setPower(0.4);
            }
            leftFrontDrive.setPower(0.0);
            rightFrontDrive.setPower(-0.0);
            leftBackDrive.setPower(0.0);
            rightBackDrive.setPower(-0.0);
            while (runtime.seconds() <= 10 && runtime.seconds() > 8) {
                wrist.setPosition(WRIST_FOLDED_IN);
                intake.setPosition(INTAKE_CLAMP);
                armMotor.setTargetPosition((int)(-10 * ARM_TICKS_PER_DEGREE));
                ((DcMotorEx) armMotor).setVelocity(2100);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            while (runtime.seconds() <= 12 && runtime.seconds() > 10) {
                leftFrontDrive.setPower(-0.6);
                rightFrontDrive.setPower(-0.9);
                leftBackDrive.setPower(0.6);
                rightBackDrive.setPower(0.3);
            }
            leftFrontDrive.setPower(0.0);
            rightFrontDrive.setPower(-0.0);
            leftBackDrive.setPower(0.0);
            rightBackDrive.setPower(-0.0);
            while (runtime.seconds() < 25 && runtime.seconds() > 12) {
                armMotor.setTargetPosition((int)(-10 * ARM_TICKS_PER_DEGREE));
                ((DcMotorEx) armMotor).setVelocity(2100);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wrist.setPosition(WRIST_FOLDED_IN);
                intake.setPosition(INTAKE_CLAMP);
            }
            
            /* send telemetry to the driver of the arm's current position and target position */
            telemetry.addData("Runtime: ", runtime.seconds());
            telemetry.addData("Arm Target: ", armMotor.getTargetPosition());
            telemetry.addData("Arm Current: ", armMotor.getCurrentPosition());
            telemetry.addData("Intake Status: ", intake.getPosition());
            telemetry.update();
        }
    }
}