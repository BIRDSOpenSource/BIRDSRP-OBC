@echo off

SET TARGET=main
del %TARGET%.ccspjt %TARGET%.cod %TARGET%.err %TARGET%.lst %TARGET%.sym log.* MPLABXLog.xml* %TARGET%.hex %TARGET%.cof

timeout 10
