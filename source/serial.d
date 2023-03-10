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

byte[] generateRandomArray() {
    byte[] arr;
    arr ~= [0x02, 0x04];
    foreach(i; 0..4) {
        arr ~= cast(ubyte) uniform(1, 10);
    }
    return arr;
}

void serialReceiveWorker(SerialPort serialport)
{
    bool running = true;
    while (running)
    {
        // TODO: hack, we should use OS waiting
        byte[6] buffer;
        byte[] readBytes = generateRandomArray();

        if (readBytes.length != 0)
        {
            // Send the message
            //writeln("Sending Message:" ~to!string(sendMsg));
            send(ownerTid, new immutable SerialMessage(readBytes));

        }

        receiveTimeout(-1.seconds, (OwnerTerminated o) { running = false; });
    }
}
