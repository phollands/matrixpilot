@echo off

@rem pyProjectCreator.py --help
@rem	-n, "--name", dest="name", help="specify the project name", type="string", default="MatrixPilot", metavar="MatrixPilot")
@rem	-t, "--target", dest="target", help="specify the target board", type="string", default="UDB5", metavar="UDB5")
@rem	-m, "--mod", dest="modules", help="search directory for module.mk file", action='append')
@rem	-i, "--inc", dest="includes", help="additional include files directory", default="")
@rem	-c, "--cfg", dest="config", help="specify configuration files directory", default="../Config")
@rem	-o, "--out", dest="out", help="project files output path", default="output")
@rem #	--defines", dest="defines", help="specify optional defines", action='append')

@rem -r specify project root path
@rem -i specify include only path (relative to root path)
@rem -m specify source only path (relative to root path)
@rem -p specify path (include + source) path (relative to root path)

@rem        value="%%CONFIG%%;../../MAVLink/include;../../Microchip;../../Microchip/Include" />


pyProjectCreator.py -t SIL   -o ../../build -r ../.. -c ../Config -m Config -m MatrixPilot -m libDCM -m MAVLink -m Tools/MatrixPilot-SIL -i FreeRTOS/include
pyProjectCreator.py -t PX4   -o ../../build -r ../.. -c ../Config -m Config -m MatrixPilot -m libSTM -m libDCM -m MAVLink -m Tools/MatrixPilot-SIL -i FreeRTOS/include
pyProjectCreator.py -t UDB4  -o ../../build -r ../.. -c ../Config -m Config -m MatrixPilot -m libUDB -m libDCM -m libFlashFS -m libVectorMatrix -m MAVLink -m Microchip -m Tools/MatrixPilot-SIL -i MAVLink/include -i Microchip -i Microchip/Include
pyProjectCreator.py -t UDB5  -o ../../build -r ../.. -c ../Config -m Config -m MatrixPilot -m libUDB -m libDCM -m libFlashFS -m libVectorMatrix -m MAVLink -m Microchip -m Tools/MatrixPilot-SIL -i MAVLink/include -i Microchip -i Microchip/Include
pyProjectCreator.py -t AUAV3 -o ../../build -r ../.. -c ../Config -m Config -m MatrixPilot -m libUDB -m libDCM -m libFlashFS -m libVectorMatrix -m MAVLink -m Microchip -m Tools/MatrixPilot-SIL -i MAVLink/include -i Microchip -i Microchip/Include -i FreeRTOS/include

pyProjectCreator.py -n RollPitchYaw -t UDB4  -o ../../build -r ../.. -c ../RollPitchYaw -m RollPitchYaw -m libUDB -m libDCM -m libVectorMatrix -i MAVLink/include -i Microchip -i Microchip/Include
pyProjectCreator.py -n RollPitchYaw -t UDB5  -o ../../build -r ../.. -c ../RollPitchYaw -m RollPitchYaw -m libUDB -m libDCM -m libVectorMatrix -i MAVLink/include -i Microchip -i Microchip/Include
pyProjectCreator.py -n RollPitchYaw -t AUAV3 -o ../../build -r ../.. -c ../RollPitchYaw -m RollPitchYaw -m libUDB -m libDCM -m libVectorMatrix -i MAVLink/include -i Microchip -i Microchip/Include

pyProjectCreator.py -n LedTest -t UDB4  -o ../../build -r ../.. -c ../../Tools/LedTest -m Tools/LedTest -m libUDB -i MAVLink/include -i Microchip -i Microchip/Include
pyProjectCreator.py -n LedTest -t UDB5  -o ../../build -r ../.. -c ../../Tools/LedTest -m Tools/LedTest -m libUDB -i MAVLink/include -i Microchip -i Microchip/Include
pyProjectCreator.py -n LedTest -t AUAV3 -o ../../build -r ../.. -c ../../Tools/LedTest -m Tools/LedTest -m libUDB -i MAVLink/include -i Microchip -i Microchip/Include
