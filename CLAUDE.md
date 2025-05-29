# Claude Assistant Configuration

This file contains configurations and information for Claude Code to reference when working with this project.

## Project Overview

This is a soil moisture monitoring system using ESP32 microcontroller. The system includes:
- Soil moisture sensing
- Valve control
- Button interface
- LCD display
- Wireless communication (WiFi and ESP-NOW)
- Data logging and management
- Web interface

## Useful Commands

### Build and Flash
```bash
idf.py build
idf.py -p PORT flash
```

### Monitor Serial Output
```bash
idf.py -p PORT monitor
```

### Clean Build
```bash
idf.py fullclean
```

### Build Specific Target
```bash
idf.py set-target esp32
idf.py build
```

## Coding Standards

- Use descriptive variable and function names
- Add comments for complex logic
- Follow ESP-IDF style guidelines
- Include proper error handling
- Document public APIs

## Project Structure

- `main/`: Core application code
- `components/`: Reusable modules
- `spiffs_image/`: Web interface files
- `partitions/`: Partition table configurations