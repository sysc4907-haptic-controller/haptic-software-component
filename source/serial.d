import std.stdio;
import std.algorithm : max;
import std.concurrency : ownerTid, receiveTimeout, send, OwnerTerminated;
import std.datetime : seconds;

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

void serialReceiveWorker(SerialPort serialport)
{
    bool running = true;
    while (running)
    {
        // TODO: hack, we should use OS waiting
        byte[128] buffer;
        byte[] readBytes = cast(byte[]) serialport.read(buffer);

        if (readBytes.length != 0)
        {
            send(ownerTid, new immutable SerialMessage(readBytes));
        }

        receiveTimeout(-1.seconds, (OwnerTerminated o) { running = false; });
    }
}
