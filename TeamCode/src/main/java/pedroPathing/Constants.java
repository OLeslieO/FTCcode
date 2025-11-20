package pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;

import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()

         .useSecondaryDrivePIDF(true)

        .mass(10)
        .forwardZeroPowerAcceleration(-33.45111)
        .lateralZeroPowerAcceleration(-58.998)
        .translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0, 0.0005, 0.02))
        .headingPIDFCoefficients(new PIDFCoefficients(0.6, 0, 0.01, 0.02))
        .drivePIDFCoefficients(new FilteredPIDFCoefficients(1,0.0,0.015,0.6,0.019))
        .centripetalScaling(0.0006);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.5
            , 2.5);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                /* other builder steps */
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .xVelocity(82.093178)
            .yVelocity(69.54082);

            //.rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)

    //不确定odo是不是这个"goBILDA Pinpoint Odometry Computer"
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(1.57) //这个要改，应该就是之前测过的odo的offsetx和offsety，单位是inch
            .strafePodX(4.72)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

}
