# Embed the server root certificate into the final binary
#
# (If this was a component, we would set COMPONENT_EMBED_TXTFILES here.)
idf_component_register(SRCS "https_request_example_main.cpp" "time_sync.c" "TempSensor.cpp"
                    INCLUDE_DIRS "include"
                    EMBED_TXTFILES server_root_cert.pem local_server_cert.pem)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
