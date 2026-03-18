# Default recipe - show available commands
default:
    @just --list

clean target:
    rm -rf builds/{{target}}

# Build outlaw gen3 firmware into builds/outlaw
outlaw:
    west build -b outlaw_gen3 apps/outlaw -p auto --build-dir builds/outlaw

outlaw-433:
    west build -b outlaw_gen3 apps/outlaw -p auto --build-dir builds/outlaw-433 -- -DCONFIG_LICENSED_FREQUENCY=y

# Build hunter receiver firmware into builds/deputy
hunter:
    west build -b hunter apps/hunter -p auto --build-dir builds/hunter

hunter-433:
    west build -b hunter apps/hunter -p auto --build-dir builds/hunter-433 -- -DCONFIG_LICENSED_FREQUENCY=y

# Flash with ST-Link
# Usage: just sflash outlaw | just sflash hunter
sflash target:
    west flash --build-dir builds/{{target}}

# Flash with J-Link
# Usage: just jflash outlaw | just jflash hunter
jflash target:
    west flash --build-dir builds/{{target}} --runner=jlink
