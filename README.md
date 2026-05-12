# Digital Voice Modem SDR

## NOTE: THIS PROJECT IS EXTREMELY ALPHA, AND WILL LIKELY HAVE PROBLEMS OR MAY STRAIGHT NOT FUNCTION, IF YOU CONTINUE, HEED THESE WORDS: THERE BE DRAGONS.

The DVM SDR provides the SDR implementation of a mixed-mode DMR, P25 and/or NXDN or dedicated-mode DMR, P25 or NXDN repeater system. The SDR; is the portion of a complete Over-The-Air modem implementation that connects directly to an appropriate SDR and performs the actual reception and creation of the digitial waveforms.

Please feel free to reach out to us for help, comments or otherwise, on our Discord: https://discord.gg/3pBe8xgrEz

## Building

This project utilizes CMake for its build system. (All following information assumes familiarity with the standard Linux make system.)

The DVM Host software requires the library dependancies below. Generally, the software attempts to be as portable as possible and as library-free as possible. A basic GCC/G++ install, with libasio and ncurses is usually all that is needed to compile.

### Dependencies

If you want detailed stacktrace output on a crash, for compilation ensure `libdw-dev` is also installed. (`apt-get install libdw-dev`). For runtime you will need the `elfutils` package to be installed. (`apt-get install elfutils`).

### Build Instructions

1. Clone the repository. `git clone https://github.com/DVMProject/dvmbbsdr.git`
2. Switch into the "dvmbbsdr" folder. Create a new folder named "build" and switch into it.
   ```
   # cd dvmbbsdr
   dvmbbsdr # mkdir build
   dvmbbsdr # cd build
   ```
3. Run CMake with any specific options required. (Where [options] is any various compilation options you require.)
   ```
   dvmbbsdr/build # cmake [options] ..
   ...
   -- Build files have been written to: dvmbbsdr/build
   dvmbbsdr/build # make
   ```
4. [OPTIONAL] Install binaries (it is *highly* recommended to not run DVM out of the build folder).
    1. Tarball Install
        1. Run build finalization.
        ```
        dvmbbsdr/build # make strip
        dvmbbsdr/build # make tarball
        ```
        2. After `make tarball` completes file named `dvmbbsdrt_R04Gxx_<arch>.tar.gz` should be created. Run the following command to install:
        ```
        dvmbbsdr/build # sudo tar xvzf dvmbbsdr_R04Gxx_<arch>.tar.gz -C /opt
        ```
    2. old_install Install
        1. Run build finalization.
        ```
        dvmbbsdr/build # make strip
        ```        
        2. Install build.
        ```
        dvmbbsdr/build # sudo make old_install
        ```        

## License

This project is licensed under the GPLv2 License - see the [LICENSE.md](LICENSE.md) file for details. Use of this project is intended, for amateur and/or educational use ONLY. Any other use is at the risk of user and all commercial purposes is strictly discouraged.
