import std.stdio : write, writeln, readln, stderr;
import std.string : fromStringz;
import std.datetime : seconds;
import std.variant : Variant;
import std.concurrency : receiveTimeout, receive, spawn, Tid, thisTid, send;
import std.file;
import std.exception : enforce;
import std.conv : to;
import std.format;
import std.math;
import std.math.rounding : round;
import kaleidic.lubeck : mldivide, inv;
import mir.ndslice;

import bindbc.sdl;
import bindbcLoader = bindbc.loader.sharedlib;
import serialport : SerialPort;

import serial : serialReceiveWorker, SerialMessage;
import sim;
import pid;

// Motor-related Values
private double noLoadRpm = 7600;
private double motorPeakVoltage = 30;
double motorTorqueConstant = 248.1609513;

//TODO: GET THE RIGHT COORDS
//units: PIXELS
const double Y_BOTTOM_LINKAGE = 1584;
const double X_LEFT_ENCODER = 465;
const double X_RIGHT_ENCODER = 915;

//units: PIXELS
const double LEN_BOTTOM_ARMS = 700;
const double LEN_TOP_ARMS = 900;
const double LEN_BETWEEN_SHAFTS = 450;


class InvalidAnglesException : Exception
{
    this(string msg) @safe pure nothrow @nogc
    {
        super(msg);
    }
}

struct ValuesFromInitialAngles
{
    PositionVector pos;
    double theta3;
    double theta4;
}

auto calculateRequiredCurrent(double sensorReading)
{
    float sensitivity = 100.0 / 500.0;
    float Vref = 2500;
    float voltage = 4.88 * sensorReading;

    return (voltage - Vref) * sensitivity;
}

auto inverseTransposeJacobian(double theta1, double theta2, double theta3, double theta4)
{
    auto jacobian = slice!double(2, 2);
    auto row0 = jacobian[0];
    auto row1 = jacobian[1];
    //This is the transpose Jacobian
    row0[0] = (LEN_BOTTOM_ARMS * sin(theta1 - theta3) * sin(theta4)) / (sin(theta3 - theta4));
    row1[0] = (LEN_BOTTOM_ARMS * sin(theta4 - theta2) * sin(theta3)) / (sin(theta3 - theta4));
    row0[1] = (LEN_BOTTOM_ARMS * sin(theta1 - theta3) * cos(theta4)) / (sin(theta3 - theta4));
    row1[1] = (LEN_BOTTOM_ARMS * sin(theta4 - theta2) * cos(theta3)) / (sin(theta3 - theta4));
    //returning the inverse of teh transpose jacobian
    return jacobian.inv;
}

ValuesFromInitialAngles getValuesFromInitialAngles(double theta1, double theta2,
        int prevY, VelocityVector v)
{
    double leftArmJointX = X_LEFT_ENCODER + LEN_BOTTOM_ARMS * cos(theta1);
    double rightArmJointX = X_RIGHT_ENCODER + LEN_BOTTOM_ARMS * cos(theta2);
    double leftArmJointY = Y_BOTTOM_LINKAGE - LEN_BOTTOM_ARMS * sin(theta1);
    double rightArmJointY = Y_BOTTOM_LINKAGE - LEN_BOTTOM_ARMS * sin(theta2);

    double v1 = (rightArmJointY - leftArmJointY) / (leftArmJointX - rightArmJointX);
    double v2 = (leftArmJointX * leftArmJointX + leftArmJointY * leftArmJointY
            - rightArmJointX * rightArmJointX - rightArmJointY * rightArmJointY) / (
            2 * (leftArmJointX - rightArmJointX));
    //double v2 = (-(LEN_BOTTOM_ARMS*LEN_BOTTOM_ARMS) + rightArmJointX*rightArmJointX + rightArmJointY*rightArmJointY) / (2*(rightArmJointX - leftArmJointX));
    double v3 = 1.0 + v1 * v1;
    double v4 = 2.0 * (v1 * v2 - v1 * leftArmJointX - leftArmJointY);

    double v5 = leftArmJointX * leftArmJointX - LEN_TOP_ARMS * LEN_TOP_ARMS +
                leftArmJointY * leftArmJointY - 2.0 * v2 * leftArmJointX + v2 * v2;

    //double v5 = LEN_BOTTOM_ARMS*LEN_BOTTOM_ARMS*LEN_TOP_ARMS*LEN_TOP_ARMS - 2.0*v2*leftArmJointX + v2*v2;

    //stderr.writeln("TEST:" ~ " | v1: " ~ to!string(v1) ~ " | v2: " ~ to!string(v2) ~ " | v3: " ~ to!string(v3) ~ " | v4: " ~ to!string(v4) ~ " | v5: " ~ to!string(v5));

    double det = v4 * v4 - 4 * v3 * v5;
    ValuesFromInitialAngles ret;

    if (det < 0)
    {
        if(det > -0.001){
            det = 0;
        } else {
            throw new InvalidAnglesException("Impossible Angles Given");
        }
    }
    double endY = (-v4 - sqrt(det)) / (2.0 * v3);
    double endX = v1 * endY + v2;

    //Calculating theta3 and theta4 ADD PI
    double theta3 = atan(abs(endY - leftArmJointY) / abs(endX - leftArmJointX));
    if (endX < leftArmJointX)
    {
        theta3 = 2 * PI - theta3;
    }
    double theta4 = PI - atan(abs(endY - rightArmJointY) / abs(endX - rightArmJointX));
    if (endX > rightArmJointX)
    {
        theta4 = 2 * PI - theta4;
    }

    ret.pos = new PositionVector(cast(int) round(endX), cast(int) round(endY));
    ret.theta3 = theta3;
    ret.theta4 = theta4;
    return ret;
}

int main(string[] args)
{
    if (args.length < 2)
    {
        stderr.writeln("usage: project-monitor /path/to/port");
        return 1;
    }

    auto sdlLoadResult = loadSDL();

    if (sdlLoadResult != sdlSupport)
    {
        writeln("Failed to load SDL.");

        foreach (error; bindbcLoader.errors)
        {
            writeln("Loader error: ", fromStringz(error.error), ": ", fromStringz(error.message));
        }
        return 1;
    }

    auto sdlTTFLoadResult = loadSDLTTF();

    if (sdlTTFLoadResult != sdlTTFSupport)
    {
        writeln("Failed to load SDL TTF.");

        foreach (error; bindbcLoader.errors)
        {
            writeln("Loader error: ", fromStringz(error.error), ": ", fromStringz(error.message));
        }
        return 1;
    }

    auto error = SDL_Init(SDL_INIT_VIDEO);

    // Create 1300x1000 window - size can be adjusted in future
    const auto windowWidth = 1300;
    const auto windowHeight = 1000;

    auto window = SDL_CreateWindow("2-DOF Hybrid Haptic Device",
            SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, windowWidth, windowHeight, 0);

    if (!window)
    {
        writeln("Could not create window: ", SDL_GetError());
        return 1;
    }

    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");

    auto renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    if (!renderer)
    {
        writeln("Could not create renderer: ", SDL_GetError());
        return 1;
    }

    scope (exit)
    {
        SDL_DestroyRenderer(renderer);
    }

    shared serialport = new shared SerialPort(args[1], 115200);
    // TODO: See if we can get this to work
    // scope (exit)
    // {
    //     serialport.close();
    // }

    spawn(&serialReceiveWorker, serialport);

    auto black = SDL_Color(0, 0, 0);
    auto red = SDL_Color(255, 0, 0);
    auto blue = SDL_Color(0, 0, 255);
    auto orange = SDL_Color(255, 165, 0);

    // List of the elements a part of the system (walls, fields, surfaces)
    SimulationElement[] elements = [
        //Honey
        new ViscousElement(292, 548, 144, 259, orange, 0.7),

        //Magnet Fields
        //Note: Strengths may need to be raised a ton or lowered a ton. I honestly dont know.
        new MagnetFieldElement(918, 607, 32, 30, red, 1000,
                MagnetFieldElement.POLARITY.PUSH),
        new MagnetFieldElement(918, 679, 32, 30, blue, 1000,
                MagnetFieldElement.POLARITY.PULL),

        //Outside Walls
        new ImpassableElement(289, 545, 249, 30, black),
        new ImpassableElement(535, 395, 30, 180, black),
        new ImpassableElement(535, 395, 323, 30, black),
        new ImpassableElement(855, 395, 30, 153, black),
        new ImpassableElement(855, 544, 224, 30, black),
        new ImpassableElement(1076, 544, 30, 293, black),
        new ImpassableElement(289, 807, 790, 30, black),
        new ImpassableElement(289, 545, 30, 265, black),

        //Inside Walls
        new ImpassableElement(678, 441, 30, 146, black),
        new ImpassableElement(572, 688, 232, 30, black),
        new ImpassableElement(918, 637, 32, 42, black),

        //Magnet-Walls
        new ImpassableElement(918, 607, 32, 30, red),
        new ImpassableElement(918, 679, 32, 30, blue)
    ];

    // Create the end effector representation in simulation
    int mouseX, mouseY;
    SDL_GetMouseState(&mouseX, &mouseY);
    EndEffector endEffector = new EndEffector(mouseX, mouseY);

    double theta1, theta2;
    int xForceSensorReading, yForceSensorReading, leftCurrentSensorReading,
        rightCurrentSensorReading;

    theta2 = acos((cast(double)(
            2 * LEN_TOP_ARMS - LEN_BETWEEN_SHAFTS)) / (2 * LEN_BOTTOM_ARMS));
    theta1 = PI - theta2;

    xForceSensorReading = 0;
    xForceSensorReading = 0;

    leftCurrentSensorReading = 0;
    rightCurrentSensorReading = 0;

    auto background = SDL_Rect(0, 0, 1300, 1000);

    // Create visual representation of the end effector (green square)
    // auto cursor = SDL_Rect(endEffector.x, endEffector.y, 10, 10);

    //bool ledOn = false;
    bool mouseMode = false;

    Pid leftMotorController = new Pid(0);
    Pid rightMotorController = new Pid(0);

    bool initialize = true;

    writeln("TEST");

    // Event Loop
    while (true)
    {
        if (initialize) {
            byte[3] initializeMsg = [0x00, 0x00, 0x00];
            //serialport.write(initializeMsg);
            initialize = false;

            writeln("SENT INITIALIZE MESSAGE");
        }

        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT || (event.type == SDL_KEYDOWN
                    && event.key.keysym.sym == SDLK_q))
            {
                return 0;
            }

            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_g)
            {
                //serialport.write([ledOn ? 1: 2]);
                mouseMode = !mouseMode;
            }
        }
        /*
        SENSOR IDS:
        -------------------
        LEFT ENCODER:  0x0
        RIGHT ENCODER: 0x1

        LEFT CURRENT SENSOR: 0x2
        RIGHT CURRENT SENSOR: 0x3

        X-FORCE SENSOR: 0x4
        Y-FORCE SENSOR: 0x5

        Data: 32-bit int
        */
        receiveTimeout(-1.seconds, (immutable SerialMessage message) {
            enforce(message.message.length != 6, "Serial Message should be 6 bytes long | Message: " ~to!string(message.message));
            enforce(message.message[0] != 0x02, "Serial Message be a Sensor Message | Message: " ~to!string(message.message));
            enforce(message.message[1] != 0x04, "Serial Message should have 4 bytes of data | Message: " ~to!string(message.message));
            auto id = message.message[2];
            auto ch1 = message.message[3];
            auto ch2 = message.message[4];
            auto dir = message.message[5] ? -1 : 1;
            enforce((ch1 | ch2) < 0, "Serial data should have 2 bytes");
            int data = ((ch1 << 8) + (ch2 << 0));


            writeln("Received Data: " ~ to!string(message.message));
            // LEFT ENCODER
            if (id == 0x0)
            {
                theta1 += data * (PI / 180.0) * dir;
            }
            //RIGHT ENCODER
            else if (id == 0x1)
            {
                theta2 += data * (PI / 180.0) * dir;
            }

            // LEFT CURRENT SENSOR
            if (id == 0x2)
            {
                leftCurrentSensorReading = data * dir;
                writeln("LEFT CURRENT SENSOR");
            }
            //RIGHT CURRENT SENSOR
            else if (id == 0x3)
            {
                rightCurrentSensorReading = data * dir;
            }

            // X FORCE SENSOR
            if (id == 0x4)
            {
                xForceSensorReading = data * dir;
            }
            //Y FORCE SENSOR
            else if (id == 0x5)
            {
                yForceSensorReading = data * dir;
            }

        });

        // Clear render with a white background
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
        SDL_RenderFillRect(renderer, &background);
        SDL_RenderClear(renderer);

        // Draw each simulation element
        foreach (SimulationElement element; elements)
        {
            element.draw(renderer);

            // If a collision is detected between endeffector and a sim. element,
            //  adjust the cursor location such that it doesn't disconnect with the visual element
            if (endEffector.detectCollision(element) && mouseMode)
            {
                SDL_WarpMouseInWindow(window, endEffector.x, endEffector.y);
            }
        }
        PositionVector endEffectorPosFromAngles;
        double theta3;
        double theta4;
        ValuesFromInitialAngles endPointsAndThetas;

        if (mouseMode)
        {
            // Update the end effector's data (position, time)
            SDL_GetMouseState(&mouseX, &mouseY);
            endEffector.update(mouseX, mouseY);
        }
        else
        {
            try
            {
                endPointsAndThetas = getValuesFromInitialAngles(theta1, theta2,
                        endEffector.y, endEffector.currVelocity);
            }
            catch (InvalidAnglesException e)
            {
                stderr.writeln("ERROR CALCULATING END POINT FOR THE FOLLOWING THETA_1: " ~ to!string(
                        theta1) ~ " | THETA_2: " ~ to!string(theta2));
                continue;
            }
            endEffectorPosFromAngles = endPointsAndThetas.pos;
            theta3 = endPointsAndThetas.theta3;
            theta4 = endPointsAndThetas.theta4;

            endEffector.update(endEffectorPosFromAngles);
        }

        // This setup is pretty janky, we definitely wanna change it to converging forces eventually
        auto currentPosition = new PositionVector(endEffector.x, endEffector.y);
        auto totalForce = new ForceVector(0, 0);
        auto currentForce = endEffector.calculateForce();

        // Draw the cursor
        SDL_SetRenderDrawColor(renderer, 0, 255, 0, SDL_ALPHA_OPAQUE);
        endEffector.draw(renderer);

        // "Load" all elements to the render
        SDL_RenderPresent(renderer);

        // Check the magnetic forces
        foreach (SimulationElement element; elements)
        {
            if (cast(MagnetFieldElement) element)
            {
                auto force = element.force(currentPosition, currentForce);
                totalForce.x += force.x;
                totalForce.y += force.y;
                currentForce.x += force.x;
                currentForce.y += force.y;
            }
        }

        // Check the reactive forces from viscous elements
        foreach (SimulationElement element; elements)
        {
            if (cast(ViscousElement) element)
            {
                auto force = element.force(currentPosition, currentForce);
                totalForce.x += force.x;
                totalForce.y += force.y;
                currentForce.x += force.x;
                currentForce.y += force.y;
            }
        }

        // Check normal forces from impassable elements
        foreach (SimulationElement element; elements)
        {
            if (cast(ImpassableElement) element)
            {
                auto force = element.force(currentPosition, currentForce);
                totalForce.x += force.x;
                totalForce.y += force.y;
            }
        }
        if (!totalForce.isEmpty())
            stderr.writeln("Horizontal Force (N): " ~ to!string(
                    totalForce.x) ~ " | Vertical Force(N): " ~ to!string(totalForce.y));

        if (!mouseMode)
        {
            auto forceMatrix = [totalForce.x, totalForce.y].sliced(2, 1);
            auto torques = mldivide(inverseTransposeJacobian(theta1, theta2,
                    theta3, theta4), forceMatrix);

            // Update target current according to required torques to simulate forces
            leftMotorController.updateTarget(torques[0][0] / motorTorqueConstant);
            rightMotorController.updateTarget(torques[1][0] / motorTorqueConstant);

            // Calculate current readings from sensors
            double leftCurrentReading = calculateRequiredCurrent(leftCurrentSensorReading);
            double rightCurrentReading = calculateRequiredCurrent(rightCurrentSensorReading);

            writeln("Sensor Reading: " ~ to!string(leftCurrentSensorReading));
            writeln("Current:"~to!string(leftCurrentReading));

            // Calculate the control signal according to error
            double leftControlSignal = leftMotorController.calculateControlSignal(
                leftCurrentReading);
            double rightControlSignal = rightMotorController.calculateControlSignal(
                rightCurrentReading);

            // Define message type and size
            byte[1] msg_type = [0x01];
            byte[1] msg_size = [0x18];

            // Define motor IDs
            ubyte leftMotorId = 0x01;
            ubyte rightMotorId = 0x02;

            // Create message for left motor and send
            ubyte power = to!ubyte(abs(leftControlSignal));

            ubyte sign = to!ubyte(sgn(leftControlSignal) == 1 ? 0 : 1);
            byte[3] leftMotor_msg = [leftMotorId, power, sign];

            /*serialport.write(msg_type);
            serialport.write(msg_size);
            serialport.write(leftMotor_msg);

            writeln("?");

            // Create message for right motor and send
            power = to!ubyte(abs(rightControlSignal));
            sign = to!ubyte(sgn(rightControlSignal) == 1 ? 0 : 1);
            byte[3] rightMotor_msg = [rightMotorId, power, sign];

            serialport.write(msg_type);
            serialport.write(msg_size);
            serialport.write(rightMotor_msg);*/

            writeln("SENT");
        }
    }
}
