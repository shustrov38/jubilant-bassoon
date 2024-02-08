#!/bin/bash

cmake --build _build --parallel -v 2>&1 | tee make.log
EXIT_CODE=${PIPESTATUS[0]}
if [ $EXIT_CODE -ne 0 ]; then
    exit $EXIT_CODE
fi

cmake --install _build 2>&1 | tee install.log
exit $EXIT_CODE