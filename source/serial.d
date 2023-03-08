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
        byte[16] buffer;
        byte[] readBytes = cast(byte[]) serialport.read(buffer);
        byte[] temp;
        byte[] sendMsg;
        ulong remainingElements = -1;

        while (true)
        {
            // If current buffer is fully read, read a new buffer
            if (readBytes.length == 0)
            {
                readBytes = cast(byte[]) serialport.read(buffer);
            }

            // If an end index has not been defined (i.e., starting from 0th index with no fragment)
            if (remainingElements == -1)
            {

                // Find length of message at 2nd bit
                int length = readBytes[1];

                // If the length of message is greater than the buffer size
                if (length > readBytes.length)
                {
                    // Store the buffer contents into a temp array
                    temp = readBytes[0 .. readBytes.length];

                    // Calculate # of missing elements needed from next buffer
                    remainingElements = length - readBytes.length;

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
                    // Concatenate previous fragment with the rest of message from current buffer
                    sendMsg = temp ~ readBytes[0 .. remainingElements];

                    // Re-assign buffer to be without message
                    readBytes = readBytes[remainingElements .. readBytes.length];

                    // Empty fragment
                    temp = [];

                    // Denote no longer a fragment waiting
                    remainingElements = -1;
                }

                // If there is a message to send
                if (sendMsg.length != 0)
                {
                    // Send the message
                    send(ownerTid, new immutable SerialMessage(sendMsg));

                    // Clear after sending
                    sendMsg = [];
                }
            }

            if (readBytes.length != 0)
            {
                send(ownerTid, new immutable SerialMessage(readBytes));
            }

            receiveTimeout(-1.seconds, (OwnerTerminated o) { running = false; });
        }
    }
}
