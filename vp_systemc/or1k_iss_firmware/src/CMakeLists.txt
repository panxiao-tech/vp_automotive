ADD_EXECUTABLE(ecu_firmware
			../../comm/config_hw.h
			bitutils.c
			adc.h
			adc.c
			pwm.h
			pwm.c
                        main.c
                        orsocdef.h
                            )


target_link_libraries(ecu_firmware m)
