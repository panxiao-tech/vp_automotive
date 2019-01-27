ADD_EXECUTABLE(throttle_motor_lsf_tb
../../src/sca_dcmotor_lsf.h
sc_top_motor_lsf.h
main.cpp
)
target_link_libraries( throttle_motor_lsf_tb
 ${SYSTEMC_LIBRARIES}
${SYSTEMCAMS_LIBRARIES}
)
