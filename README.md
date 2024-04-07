# jubilant-bassoon

## Build 
For builing source code use [build.sh](./build.sh) and [incremental_build.sh](./incremental_build.sh) scripts. The first one will install all dependencies. The second one will incrementally build from existing cmake cache.

```bash
./build.sh <install path> <mode>

# at the first time
./build.sh ../install Release

# at other times
./incremental_build.sh
```

## Running command line app
For running a command line application, just run the installed binary.
```bash
./jubilant-bassoon
```
For saving logs of calculated variables, provide env `ACV_CSV_LOG_FILE_NAME` containing path to log file.
```bash
ACV_CSV_LOG_FILE_NAME=<path to result csv> ./jubilant-bassoon
```