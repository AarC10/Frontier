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
    west build -b outlaw_evo apps/outlaw -p auto --build-dir builds/outlaw-evo -- -DOVERLAY_CONFIG=dts/gen3_sx1262.overlay

# Build Outlaw EVO firmware with 433 MHz support
outlaw-evo-433:
    west build -b outlaw_evo apps/outlaw -p auto --build-dir builds/outlaw-evo-433 -- -DOVERLAY_CONFIG=dts/gen3_sx1262.overlay -DCONFIG_LICENSED_FREQUENCY=y

# Build hunter receiver firmwarey
hunter:
    west build -b hunter apps/hunter -p auto --build-dir builds/hunter

# Build hunter receiver firmware with 433 MHz support
hunter-433:
    west build -b hunter apps/hunter -p auto --build-dir builds/hunter-433 -- -DCONFIG_LICENSED_FREQUENCY=y

marshal:
    west build -b marshal apps/marshal -p auto --build-dir builds/marshal -DZEPHYR_SCA_VARIANT=dtdoctor

# Flash with ST-Link
# Usage: just sflash outlaw | just sflash hunter
sflash target:
    west flash --build-dir builds/{{target}}

# Flash with J-Link
# Usage: just jflash outlaw | just jflash hunter
jflash target:
    west flash --build-dir builds/{{target}} --runner=jlink
