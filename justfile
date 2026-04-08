# Default recipe - show available commands
default:
    @just --list

clean target:
    rm -rf builds/{{target}}

# Build Outlaw Gen 3 firmware
outlaw:
    west build -b outlaw_gen3 apps/outlaw -p auto --build-dir builds/outlaw

# Build Outlaw Gen 3 firmware with 433 MHz support
outlaw-433:
    west build -b outlaw_gen3 apps/outlaw -p auto --build-dir builds/outlaw-433 -- -DCONFIG_LICENSED_FREQUENCY=y

# Build Outlaw EVO firmware
outlaw-evo:
    west build -b outlaw_gen3 apps/outlaw -p auto --build-dir builds/outlaw-evo -- -DDTC_OVERLAY_FILE=dts/evo.overlay

# Build Outlaw EVO firmware with 433 MHz support
outlaw-evo-433:
    west build -b outlaw_gen3 apps/outlaw -p auto --build-dir builds/outlaw-evo-433 -- -DDTC_OVERLAY_FILE=dts/evo.overlay -DCONFIG_LICENSED_FREQUENCY=y

# Build hunter receiver firmwarey
hunter:
    west build -b hunter_gen2 apps/hunter -p auto --build-dir builds/hunter

# Build hunter receiver firmware with 433 MHz support
hunter-433:
    west build -b hunter_gen2 apps/hunter -p auto --build-dir builds/hunter-433 -- -DCONFIG_LICENSED_FREQUENCY=y

# Build hunter receiver firmwarey
hunter-old:
    west build -b hunter apps/hunter -p auto --build-dir builds/hunter-old

# Build hunter receiver firmware with 433 MHz support
hunter-old-433:
    west build -b hunter apps/hunter -p auto --build-dir builds/hunter-old-433 -- -DCONFIG_LICENSED_FREQUENCY=y


marshal:
    west build -b marshal apps/marshal -p auto --build-dir builds/marshal -DZEPHYR_SCA_VARIANT=dtdoctor

fm-radio:
    west build -b nucleo_wb55rg apps/fm_radio -p auto --build-dir builds/fm_radio -DZEPHYR_SCA_VARIANT=dtdoctor


marshal-sim:
    west build -b native_sim apps/marshal -p auto --build-dir builds/marshal-sim

outlaw-sim:
    west build -b native_sim apps/outlaw -p auto --build-dir builds/outlaw-sim

hunter-sim:
    west build -b native_sim apps/hunter -p auto --build-dir builds/hunter-sim

# Run native sim targets
run-marshal-sim:
    ./builds/marshal-sim/zephyr/zephyr.exe

run-outlaw-sim:
    ./builds/outlaw-sim/zephyr/zephyr.exe

run-hunter-sim:
    ./builds/hunter-sim/zephyr/zephyr.exe

sim-stream input format="openrocket":
    python tools/sim_stream.py {{input}} --format {{format}}

# Flash with ST-Link
# Usage: just sflash outlaw | just sflash hunter
sflash target:
    west flash --build-dir builds/{{target}}

# Flash with J-Link
# Usage: just jflash outlaw | just jflash hunter
jflash target:
    west flash --build-dir builds/{{target}} --runner=jlink
