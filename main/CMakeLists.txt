idf_component_register(
    SRCS "app_main.c"
    REQUIRES
    	app_hap_setup_payload
	app_wifi
	button
	esp_hap_core
    	esp_hap_apple_profiles
    	esp_hap_extras
	DHT22)

