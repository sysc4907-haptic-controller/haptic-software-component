import std.stdio;
import std.algorithm : max;
import std.concurrency : ownerTid, receiveTimeout, send, OwnerTerminated;
import std.datetime : seconds;
import std;

import core.stdc.stdio : perror;

import serialport : SerialPort;

// Wrapper Class for messages sent through serial
immutable class SerialMessage
{
    byte[] message;

    this(byte[] message)
    {
        this.message = message.idup;
    }
}

// Simulate current sensor readings from Arduino
byte[] generateRandomArray()
{
    byte[] arr;
    arr ~= [0x02, 0x04];
    arr ~= cast(ubyte) uniform(1, 2);
    foreach (i; 0 .. 2)
    {
        arr ~= cast(ubyte) uniform(1, 5);
    }
    arr ~= cast(ubyte) uniform(0, 1);
    return arr;
}

void serialReceiveWorker(SerialPort serialport)
{
    bool running = true;

    byte[16] buffer;
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
                send(ownerTid, new immutable SerialMessage(temp));
                temp = [];
            }

            receiveTimeout(-1.seconds, (OwnerTerminated o) { running = false; });
        }
    }
}
