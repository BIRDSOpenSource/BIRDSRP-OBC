@echo off

Rem Change path only internally to this batch file
setlocal

Rem PATH variables for PIC C Compiler
SET PATH=%PATH%;C:\Program Files (x86)\PICC

SET TARGET=main

SET CFLAGS=+FH +Y9 +EA -E +DF +GEM=""
REM +FH : Selects PCH compiler
REM +Y9 : Selects optimization level
REM +EA : Enable all errors
REM -E  : Show only the first error
REM +DF : Enable debug
REM +EX : Enable gcc error/warning output format

Ccsc.exe %CFLAGS% %TARGET%.c

type %TARGET%.err

timeout 10
