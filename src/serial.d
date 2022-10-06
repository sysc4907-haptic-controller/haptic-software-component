import std.stdio;
import std.algorithm : max;

import core.sys.posix.sys.select;
import core.stdc.stdio : perror;

import serialport : SerialPortNonBlk;

// Wrapper Class for messages sent through serial
immutable class SerialType {
    string message;

    this(string message) {
        this.message = message;
    }

    public string toStringz() {
        return this.message;
    }
}

void setupSerial(string[] args) {
}

void sendSerial(string msg) {
    //TODO: Send something over serial
}

SerialType getSerial(string[] args) {
    return new SerialType("placeHolder");
    //TODO: Setup serial receive
    /*
    // If the user didn't provide the path to the port, print usage and exit
    if (args.length < 2)
    {
        stderr.writeln("usage: project-monitor /path/to/port");
        return 1;
    }

    // Open the port at 9600 baud
    auto port = new SerialPortNonBlk(args[1], 9600);
    // On scope exit, close the handle to the port
    scope (exit) port.close();

    // Ask the device what the current state is
    port.write(['?']);

    fd_set fds; // Set of file descriptors to pass to select
    int ready; // Return value of select
    int stdin_fd = stdin.fileno(); // File descriptor of stdin
    int port_fd = port.handle(); // File descriptor of the port
    int nfds = max(stdin_fd, port_fd) + 1; // nfds parameter of select
    while (true)
    {
        // Initialize the file descriptor set
        FD_ZERO(&fds);
        FD_SET(stdin_fd, &fds);
        FD_SET(port_fd, &fds);

        // Call select with no timeout
        ready = select(nfds, &fds, null, null, null);

        // If there was an error, print it and exit
        if (ready == -1)
        {
            perror("select() failed");
            return 1;
        }

        // If we can read from stdin
        if (FD_ISSET(stdin_fd, &fds))
        {
            // Read 1 byte
            byte[1] buffer;
            byte[] readBytes = stdin.rawRead(buffer);

            // If the read returned length 0, stdin has closed (user closed)
            if (readBytes.length == 0)
            {
                writeln("goodbye");
                return 0;
            }

            // Get the byte (a command from the user)
            byte command = readBytes[0];

            // If it's a '0', '1', '2', or '3', the user is setting the
            // state.
            if ('0' <= command && command <= '3')
            {
                // Turn it from, for example, '0' (0x30) into 0 (0x00)
                byte newState = cast(byte) (command - 48);
                writefln("setting state to %s", newState);

                // Write the command to the port
                port.write([newState]);
            }
            // If it's 'q', exit
            else if (command == 'q')
            {
                writeln("goodbye");
                return 0;
            }
            // If it's anything else other than a newline, it's not a valid
            // command
            else if (command != '\n')
                {
                    stderr.writeln("invalid command");
                }
        }

        // If we can read from the port
        if (FD_ISSET(port_fd, &fds))
        {
            // Read 1 byte
            byte[1] buffer;
            byte[] readBytes = cast(byte[]) port.read(buffer);

            // If the read returned length 0, the port has closed (device was
            // disconnected)
            if (readBytes.length == 0)
            {
                writeln("device disconnected");
                return 2;
            }

            // Get the byte (a state value from the device)
            byte state = readBytes[0];

            // If it's 0, 1, 2, or 3, it's the current state of the device
            if (0 <= state && state <= 3)
            {
                // Tell the user what the new state is
                writefln("state is %s", state);
            }
            // If it's 'e', the device is telling us that it tried to
            // switch to an invalid state
            else if (state == 'e')
            {
                stderr.writeln("tried to switch to an invalid state");
            }
        }
    }*/
}