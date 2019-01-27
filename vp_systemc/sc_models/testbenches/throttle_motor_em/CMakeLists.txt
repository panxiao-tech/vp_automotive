ADD_EXECUTABLE(throttle_motor_em_tb
../../src/sca_dcmotor_em.h
sc_top_eln_e.h
sc_top_lsf_m.h
sc_top_motor_em.h
main.cpp
)
target_link_libraries( throttle_motor_em_tb
 ${SYSTEMC_LIBRARIES}
${SYSTEMCAMS_LIBRARIES}
)
