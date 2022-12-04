import std.string : toStringz;

import std.format : format;

version (Posix)
{
    import core.sys.posix.unistd : read, write, close;
    import core.sys.posix.fcntl : open, O_RDWR, O_NOCTTY, O_NONBLOCK;
    import core.stdc.errno;

    version (linux)
    {
        import core.sys.linux.termios;
    }
    else
    {
        import core.sys.posix.termios;
    }

    alias posixRead = read;
    alias posixWrite = write;
    alias posixClose = close;

    alias HandleType = int;
    enum nullHandle = -1;

    alias closeHandle = posixClose;
}
version (Windows)
{
    import core.sys.windows.windows;
    import core.sys.windows.winbase;

    alias HandleType = HANDLE;
    enum nullHandle = null;

    alias closeHandle = CloseHandle;
}

class InvalidStateException : Exception
{
    this(string msg) @safe pure nothrow @nogc
    {
        super(msg);
    }
}

class SysCallException : Exception
{
    this(string msg, int err) @safe pure
    {
        super(format!"%s Error number %s"(msg, err));
    }
}

shared class SerialPort
{
public:
    string port;

    HandleType _handle = nullHandle;

    this(string port, uint baudRate)
    {
        reopen(port, baudRate);
    }

    ~this()
    {
        close();
    }

    void close()
    {
        if (closed)
        {
            return;
        }
        closeHandle(cast(HandleType) _handle);
        _handle = nullHandle;
    }

    void reopen(string np, uint br)
    {
        if (!closed)
        {
            close();
        }
        port = np;
        setup(br);
    }

    string toString() const
    {
        return port;
    }

    bool closed() const @property nothrow
    {
        version (Posix)
        {
            return _handle == nullHandle;
        }
        version (Windows)
        {
            return _handle is nullHandle;
        }
    }

    void initializePort(uint baudRate)
    {
        if (closed)
        {
            throw new InvalidStateException("Port is closed.");
        }

        version (Posix)
        {
            termios opt;
            if (tcgetattr(_handle, &opt) == -1)
            {
                throw new SysCallException("tcgetattr failed.", errno);
            }

            speed_t speed;
            switch (baudRate)
            {
            case 50:
                speed = B50;
                break;
            case 75:
                speed = B75;
                break;
            case 110:
                speed = B110;
                break;
            case 134:
                speed = B134;
                break;
            case 150:
                speed = B150;
                break;
            case 200:
                speed = B200;
                break;
            case 300:
                speed = B300;
                break;
            case 600:
                speed = B600;
                break;
            case 1200:
                speed = B1200;
                break;
            case 1800:
                speed = B1800;
                break;
            case 2400:
                speed = B2400;
                break;
            case 4800:
                speed = B4800;
                break;
            case 9600:
                speed = B9600;
                break;
            case 19200:
                speed = B19200;
                break;
            case 38400:
                speed = B38400;
                break;
            case 57600:
                speed = B57600;
                break;
            case 115200:
                speed = B115200;
                break;
            case 230400:
                speed = B230400;
                break;
            case 0:
            default:
                speed = B0;
            }

            cfsetispeed(&opt, B0);
            cfsetospeed(&opt, speed);

            opt.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
            opt.c_oflag &= ~OPOST;
            opt.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
            opt.c_cc[VMIN] = 0;
            opt.c_cc[VTIME] = 0;

            opt.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
            opt.c_cflag |= CS8;

            if (tcsetattr(_handle, TCSANOW, &opt) == -1)
            {
                throw new SysCallException("tcsetattr failed.", errno);
            }
        }
        version (Windows)
        {
            DCB cfg;
            GetCommState(cast(HandleType) _handle, &cfg);

            if (cfg.BaudRate != cast(DWORD) baudRate)
            {
                cfg.BaudRate = cast(DWORD) baudRate;
            }

            if (cfg.Parity != NOPARITY)
            {
                cfg.Parity = NOPARITY;
            }

            if (cfg.StopBits != ONESTOPBIT)
            {
                cfg.StopBits = ONESTOPBIT;
            }

            if (cfg.ByteSize != 8)
            {
                cfg.ByteSize = 8;
            }

            if (!SetCommState(cast(HandleType) _handle, &cfg))
            {
                throw new SysCallException("SetCommState failed.", GetLastError());
            }
        }
    }

    void[] read(void[] buf)
    {
        // non-blocking algorithm
        if (closed)
        {
            throw new InvalidStateException("Port is closed.");
        }

        auto ptr = buf.ptr;
        auto len = buf.length;

        size_t res;

        version (Posix)
        {
            auto sres = posixRead(_handle, ptr, len);

            // no bytes for read, it's ok
            if (sres < 0)
            {
                if (errno == EAGAIN)
                {
                    sres = 0;
                }
                else
                {
                    throw new SysCallException("read failed.", errno);
                }
            }
            res = sres;
        }
        version (Windows)
        {
            uint sres;
            auto rfr = ReadFile(cast(HandleType) _handle, ptr, cast(uint) len, &sres, null);
            if (!rfr)
            {
                auto err = GetLastError();
                if (err != ERROR_IO_PENDING)
                {
                    throw new SysCallException("ReadFile failed.", err);
                }
            }
            res = sres;
        }

        return buf[0 .. res];
    }

    size_t write(const(void[]) arr)
    {
        // non-blocking algorithm
        if (closed)
        {
            throw new InvalidStateException("Port is closed.");
        }

        auto ptr = arr.ptr;
        auto len = arr.length;

        version (Posix)
        {
            ptrdiff_t res = posixWrite(_handle, ptr, len);
            if (res < 0)
            {
                if (errno == EAGAIN) // buffer filled
                {
                    res = 0;
                }
                else
                {
                    throw new SysCallException("write failed.", errno);
                }
            }
        }
        version (Windows)
        {
            uint res;
            auto wfr = WriteFile(cast(HandleType) _handle, ptr, cast(uint) len, &res, null);
            if (!wfr)
            {
                auto err = GetLastError();
                if (err == ERROR_IO_PENDING)
                {
                    res = 0;
                }
                else
                {
                    throw new SysCallException("WriteFile failed.", err);
                }
            }
        }

        return res;
    }

protected:
    void setup(uint br)
    {
        if (port.length == 0)
        {
            throw new Exception("invalid port.");
        }

        version (Posix)
        {
            _handle = open(port.toStringz(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (_handle == -1)
            {
                throw new SysCallException("open failed.", errno);
            }
        }
        version (Windows)
        {
            auto fname = `\\.\` ~ port;
            _handle = cast(shared(HandleType)) CreateFileA(fname.toStringz,
                    GENERIC_READ | GENERIC_WRITE, 0, null, OPEN_EXISTING, 0, null);

            if (_handle is INVALID_HANDLE_VALUE)
            {
                throw new SysCallException("CreateFileA failed.", GetLastError());
            }

            SetupComm(cast(HandleType) _handle, 4096, 4096);
            PurgeComm(cast(HandleType) _handle,
                    PURGE_TXABORT | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_RXCLEAR);

            COMMTIMEOUTS tm;
            tm.ReadIntervalTimeout = 0;
            tm.ReadTotalTimeoutMultiplier = 0;
            tm.ReadTotalTimeoutConstant = 0;
            tm.WriteTotalTimeoutMultiplier = 0;
            tm.WriteTotalTimeoutConstant = 0;

            if (SetCommTimeouts(cast(HandleType) _handle, &tm) == 0)
            {
                throw new SysCallException("SetCommTimeouts failed.", GetLastError());
            }
        }

        initializePort(br);
    }
}
