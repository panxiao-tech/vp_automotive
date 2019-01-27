ADD_EXECUTABLE(app_sensor_tb
../../src/sca_dtdf_app_sensor.h
sc_top.h
main.cpp
)
target_link_libraries( app_sensor_tb
 ${SYSTEMC_LIBRARIES}
${SYSTEMCAMS_LIBRARIES}
)
