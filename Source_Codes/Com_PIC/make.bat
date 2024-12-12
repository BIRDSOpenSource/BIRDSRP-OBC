@echo off

Rem Change path only internally to this batch file
setlocal

Rem PATH variables for PIC C Compiler
SET PATH=%PATH%;C:\Program Files (x86)\PICC

SET TARGET=compic_main

SET CFLAGS=+FM +Y9 +EA -E +DF
REM +FM : Selects PCM compiler
REM +Y9 : Selects optimization level
REM +EA : Enable all errors
REM -E  : Show only the first error
REM +DF : Enable debug

Ccsc.exe %CFLAGS% %TARGET%.c

type %TARGET%.err

pause
