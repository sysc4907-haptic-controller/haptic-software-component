import std.stdio;
import std.algorithm : max;
import std.concurrency : receiveTimeout, OwnerTerminated;
import std.datetime : seconds;
import std;

import core.stdc.stdio : perror;

import serialport : SerialPort;
import sensorvalueholder;

void serialReceiveWorker(SerialPort serialport, SensorValueHolder sensorValueHolder)
{
    bool running = true;

    byte[32] buffer;
    int msg_size = 8;
    byte[] readBytes;
    byte[] temp;
    int remainingElements = 0;
    while (running)
    {
        // Read from buffer
        readBytes = cast(byte[]) serialport.read(buffer);
        for (int i = 0; i < readBytes.length; i++)
        {
            if (readBytes[i] == 0x76 && readBytes[i + 1] == 0x76)
            {
                temp = [];
                remainingElements = msg_size;
            }

            if (remainingElements != 0 && temp.length != msg_size)
            {
                temp ~= readBytes[i];
                remainingElements--;
            }

            if (temp.length == msg_size)
            {
                enforce(temp.length == 8, "Serial Message should be 8 bytes long | Message: " ~to!string(temp) ~ " | Message Length: "~to!string(temp.length));
                enforce(temp[2] == 0x02, "Serial Message be a Sensor Message | Message: " ~to!string(temp));
                enforce(temp[3] == 0x04, "Serial Message should have 4 bytes of data | Message: " ~to!string(temp));
                ubyte id = temp[4];
                ubyte ch1 = temp[5];
                ubyte ch2 = temp[6];
                int dir = temp[7] ? -1 : 1;
                enforce((ch1 & ch2) >= 0, "Serial data should have 2 bytes");
                ushort data = ((ch1 << 8) | (ch2 << 0));

                // LEFT ENCODER
                if (id == 0x3)
                {
                    short x = cast(short) (data * dir);
                    sensorValueHolder.leftEncoder = x;
                }
                //RIGHT ENCODER
                else if (id == 0x4)
                {
                    short x = cast(short) (data * dir);
                    sensorValueHolder.rightEncoder = x;
                }

                // LEFT CURRENT SENSOR
                if (id == 0x1)
                {
                    short x = cast(short) (data * dir);
                    sensorValueHolder.leftCurrentSensor = x;
                }
                //RIGHT CURRENT SENSOR
                else if (id == 0x2)
                {
                    sensorValueHolder.rightCurrentSensor = cast(short) (data * dir);
                }

                /*// X FORCE SENSOR
                if (id == 0x5)
                {
                    newClass.updateXForceSensor(data);
                }
                //Y FORCE SENSOR
                else if (id == 0x6)
                {
                    .updateYForcerSensor(data);
                }*/

                temp = [];
            }

            receiveTimeout(-1.seconds, (OwnerTerminated o) { running = false; });
        }
    }
}
