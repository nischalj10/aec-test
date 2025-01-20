update -

- the echo cancellation works with SR mode, not the voice communication mode. see the config in [aec_handler.h](/main/aec_handler.h) file

this project aims to do acoustic echo cancellation (aec) with espressif's [audio frontend framework](https://docs.espressif.com/projects/esp-sr/en/latest/esp32s3/audio_front_end/README.html)

it is configured to recieve input from a mic and play it via the speaker (without echo)

hardware setup -

- seeed xiao esp32s3
- inmp 441 mems mic
- max98357a amp for audio out

pin connections can be found in [i2s_config.h](/main/i2s_config.h) file
