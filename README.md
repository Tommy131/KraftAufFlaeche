# Robotik Arduino Sources

> This repository contains the sources for the controller used by team __Kraft auf Fläche__

## Development process

### Tools

- Preferred: `PlatformIO IDE` for vscode


### Code quality assurance

```bash
# Static code analysis
pio check

# run unit tests (for UNO)
pio test -e unotest -v

# runn unit tests (for kieblitz)
pio test -e wemos_d1_mini32

# ci commands
pio ci --board=uno --lib="." --exclude="src/test" .
pio ci --board=megaatmega2560 --lib="." --exclude="src/test" .
pio ci --board=wemos_d1_mini32 --lib="." .
```
