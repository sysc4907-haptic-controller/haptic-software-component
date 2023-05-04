# haptic-software-component
The software component of the SYSC4907 Haptic Device Capstone project

## Setting up

### D/DMD/Dub

Install DMD from [here](https://dlang.org/download.html#dmd). This should also
install `dub`.

### IntelliJ

There are many editor options, but a pretty good one is
[IntelliJ](https://www.jetbrains.com/idea/) with the [IntelliJ D
Language](https://intellij-dlanguage.github.io/) plugin.

### SDL

Make sure you have SDL and SDL\_ttf installed. The easiest way to do this on
Windows is download the win32-\<architecture\> zip file (probably win32-x64 for
your machine) for SDL and SDL\_ttf, unzip them, and move the resulting dll into
the same directory as `dub.sdl`. The zip files can be found at:

* https://github.com/libsdl-org/SDL/releases
* https://github.com/libsdl-org/SDL_ttf/releases

### Lubeck/mir-lapack

On Windows, you can install Intel MKL from
[here](https://www.intel.com/content/www/us/en/developer/tools/oneapi/onemkl.html). This
will install the required libraries to somewhere like `C:\Program Files
(x86)\Intel\oneAPI\mkl\latest\lib\intel64`. If The libraries are not in that
location, you must update `dub.sdl` to point at the correct location.

For more details, see the [mir-lapack
wiki](https://github.com/libmir/mir-lapack/wiki/Link-with-CBLAS-&-LAPACK).

### Compiling/running

Run `dub` in the same directory as `dub.sdl`. For more details see the [dub
documentation](https://dub.pm/commandline.html).
