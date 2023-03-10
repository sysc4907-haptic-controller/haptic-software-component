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

void serialReceiveWorker(SerialPort serialport)
{
    bool running = true;
    while (running)
    {
        // TODO: hack, we should use OS waiting
        byte[6] buffer;
        byte[] readBytes = cast(byte[]) serialport.read(buffer);
        writeln("First Buffer Reading:" ~to!string(readBytes));
        byte[] temp;
        byte[] sendMsg;
        int length = 0;
        ulong remainingElements = -1;

        while (true)
        {
            writeln("Buffer Reading:" ~to!string(readBytes));
            if (readBytes.length == 1 && length == -1) {
                temp = readBytes;
            }

            // If current buffer is fully read, read a new buffer
            if (readBytes.length == 0 || (readBytes.length == 1 && length == -1))
            {
                if (readBytes.length == 1 && length == -1) {
                    readBytes = temp~cast(byte[]) serialport.read(buffer);
                } else {
                    readBytes = cast(byte[]) serialport.read(buffer);
                }
                writeln("New Buffer Reading:" ~to!string(readBytes));
            }

            // If an end index has not been defined (i.e., starting from 0th index with no fragment)
            if (remainingElements == -1)
            {

                // Find length of message at 2nd bit
                length = to!int(readBytes[1]) + 2;

                // If the length of message is greater than the buffer size
                if (length > readBytes.length)
                {
                    writeln("AGNAGHIHAWGOHG");
                    // Store the buffer contents into a temp array
                    temp = readBytes[0 .. readBytes.length];

                    // Calculate # of missing elements needed from next buffer
                    remainingElements = length - readBytes.length;

                    writeln("Temp:" ~to!string(temp));
                    writeln("REM:" ~to!string(remainingElements));

                    // Empty buffer to notify we need a new one
                    readBytes = [];

                    // Full message is contained within the current buffer
                }
                else
                {
                    // Copy segment of buffer containing message
                    sendMsg = readBytes[0 .. length];

                    // Re-assign buffer to be without message
                    readBytes = readBytes[length .. readBytes.length];
                }

                // Previous message was not complete - need to grab fragment from this buffer
            }
            else
            {

                // If the # of missing elements is greater than the size of the buffer
                if (remainingElements > readBytes.length)
                {
                    writeln("HELP");
                    // Concatenate the previous fragment with the new fragment
                    temp = temp ~ readBytes[0 .. readBytes.length];

                    // Calculate new # of remaining elements to get
                    remainingElements = remainingElements - readBytes.length;

                    // Empty buffer
                    readBytes = [];

                    // Can complete the fragmented message
                }
                else
                {
                    writeln("PLEAASE???");
                    // Concatenate previous fragment with the rest of message from current buffer
                    sendMsg = temp ~ readBytes[0 .. remainingElements];

                    // Re-assign buffer to be without message
                    readBytes = readBytes[remainingElements .. readBytes.length];

                    // Empty fragment
                    temp = [];

                    // Denote no longer a fragment waiting
                    remainingElements = -1;
                }
            }

            // If there is a message to send
            if (sendMsg.length != 0)
            {
                // Send the message
                writeln("Sending Message:" ~to!string(sendMsg));
                send(ownerTid, new immutable SerialMessage(sendMsg));

                // Clear after sending
                sendMsg = [];
                length = -1;
            }

            receiveTimeout(-1.seconds, (OwnerTerminated o) { running = false; });
        }
    }
}
