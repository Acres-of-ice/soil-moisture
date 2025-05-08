#!/bin/bash

usage() {
  echo "Usage: $0 [OPTIONS] [TYPE]"
  echo "OPTIONS:"
  echo "  -s, --site NAME      Set site name (implies conductor flash)"
  echo "  -w, --webapp         Include web application"
  echo "  -p, --port PORT      Set ESP port (default: auto-detect)"
  echo "  -c, --chip CHIP      Set ESP chip type (esp32, esp32s3, esp32c3, etc.)"
  echo "TYPES:"
  echo "  source           Flash Source Valve"
  echo "  drain            Flash Drain Valve"
  echo "  air              Flash Air Valve"
  echo "  aws              Flash AWS Gateway"
  echo "  gsm              Flash GSM Relay"
  echo ""
  echo "Note: If -s/--site is provided, type defaults to conductor"
  exit 1
}

# Auto-detect ESP32 port function
auto_detect_esp_port() {
  # Try to find ESP32 devices on common paths
  if [ "$(uname)" == "Darwin" ]; then
    # macOS
    DETECTED_PORT=$(ls -1 /dev/cu.SLAB_USBtoUART* /dev/cu.usbserial* /dev/tty.SLAB_USBtoUART* /dev/tty.usbserial* /dev/cu.wchusbserial* /dev/tty.wchusbserial* 2>/dev/null | head -n 1)
  else
    # Linux
    # Try common ESP32 ports
    for device in $(find /dev -name "ttyUSB*" -o -name "ttyACM*" -o -name "ttyS*" 2>/dev/null); do
      if [ -e "$device" ]; then
        DETECTED_PORT="$device"
        break
      fi
    done
  fi

  echo "$DETECTED_PORT"
}

# Auto-detect ESP chip type from sdkconfig
auto_detect_chip() {
  if [ -f "sdkconfig" ]; then
    # Check if CONFIG_IDF_TARGET is defined in sdkconfig
    local chip_type=$(grep -E "^CONFIG_IDF_TARGET=\"(.*)\"$" sdkconfig | cut -d'"' -f2)
    if [ -n "$chip_type" ]; then
      echo "$chip_type"
      return
    fi
  fi

  # Default to esp32 if detection fails
  echo "esp32"
}

PCB_TYPE=$1
export PCB_TYPE          # Export PCB_TYPE so it's available to child processes
SITE_NAME=${2:-""}       # Get site name from second argument if provided
INCLUDE_WEB_APP=${3:-""} # Web app is now third argument if provided
ESPPORT=""               # Will be auto-detected if not specified
ESP_CHIP=""              # Will be auto-detected if not specified

# Parse command line arguments
while [[ $# -gt 0 ]]; do
  case $1 in
  -h | --help)
    usage
    ;;
  -s | --site)
    SITE_NAME="$2"
    PCB_TYPE="CONDUCTOR" # Automatically set to conductor
    shift 2
    ;;
  -w | --webapp)
    INCLUDE_WEB_APP="web_app"
    shift
    ;;
  -p | --port)
    ESPPORT="$2"
    shift 2
    ;;
  -c | --chip)
    ESP_CHIP="$2"
    shift 2
    ;;
  source | drain | air | aws | gsm)
    if [ -n "$SITE_NAME" ]; then
      echo "Warning: PCB type ignored because site name was provided. Using conductor."
    else
      PCB_TYPE="${1^^}" # Convert to uppercase
    fi
    shift
    ;;
  *)
    echo "Unknown parameter: $1"
    usage
    ;;
  esac
done

# Validate required parameters
if [ -z "$PCB_TYPE" ]; then
  echo "Error: PCB type is required"
  usage
fi

case "${PCB_TYPE,,}" in # Convert to lowercase for comparison
"aws")
  PCB_TYPE="AWS"
  ;;
"gsm")
  PCB_TYPE="GSM"
  ;;
"source")
  PCB_TYPE="SOURCE_NOTE"
  ;;
"drain")
  PCB_TYPE="DRAIN_NOTE"
  ;;
"air") # Added missing air valve type
  PCB_TYPE="AIR_NOTE"
  ;;
"conductor")
  PCB_TYPE="CONDUCTOR"
  ;;
"")
  echo "Error: PCB type not specified"
  usage
  exit 1
  ;;
*)
  if [ "$SITE_NAME" ]; then
    PCB_TYPE="CONDUCTOR" # Default to conductor if site name is provided
    echo "Using CONDUCTOR type since site name was provided"
  else
    echo "Error: Invalid PCB type: $PCB_TYPE"
    usage
    exit 1
  fi
  ;;
esac

# Auto-detect or use provided port
if [ -z "$ESPPORT" ]; then
  ESPPORT=$(auto_detect_esp_port)
  if [ -z "$ESPPORT" ]; then
    echo "Error: Could not auto-detect ESP port. Please connect your device or specify the port manually with -p/--port."
    exit 1
  else
    echo "Auto-detected ESP port at: $ESPPORT"
  fi
fi
export ESPPORT # Export ESPPORT so it's available to child processes

# Auto-detect or use provided chip type
if [ -z "$ESP_CHIP" ]; then
  ESP_CHIP=$(auto_detect_chip)
  echo "Auto-detected ESP chip type: $ESP_CHIP"
else
  echo "Using specified ESP chip type: $ESP_CHIP"
fi
export ESP_CHIP # Export ESP_CHIP so it's available to child processes

# Update the binary name selection to include site name for Conductor
if [ "$PCB_TYPE" == "CONDUCTOR" ]; then
  # If no site name provided, try to get it from sdkconfig
  if [ -z "$SITE_NAME" ] && [ -f "sdkconfig" ]; then
    SITE_NAME=$(grep "CONFIG_SITE_NAME=" sdkconfig | cut -d'"' -f2)
  fi
  # If still no site name, use default
  if [ -z "$SITE_NAME" ]; then
    SITE_NAME="Igoo"
    echo -e "\033[1;33mWARNING: No site name provided. Using default: ${SITE_NAME}\033[0m"
  fi
  BINARY_NAME="${SITE_NAME}_${PCB_TYPE}.bin"
else
  BINARY_NAME="${PCB_TYPE}.bin"
fi

# Add informative message about which file will be flashed
echo -e "\033[1;32m================================\033[0m"
echo -e "\033[1;32mFlashing binary: ${BINARY_NAME}\033[0m"
echo -e "\033[1;32mUsing port: ${ESPPORT}\033[0m"
echo -e "\033[1;32mUsing chip: ${ESP_CHIP}\033[0m"
echo -e "\033[1;32m================================\033[0m"

# Set partition file and flash size based on PCB type
if [ "$PCB_TYPE" == "CONDUCTOR" ]; then
  PARTITIONS_FILE="partitions/partitions_CONDUCTOR.csv"
  PARTITION_TABLE_BIN="images/partition_table_8MB.bin"
elif [ "$PCB_TYPE" == "AWS" ]; then
  PARTITIONS_FILE="partitions/partitions_AWS.csv"
  PARTITION_TABLE_BIN="images/partition_table_8MB.bin"
elif [ "$PCB_TYPE" == "GSM" ]; then
  PARTITIONS_FILE="partitions/partitions_AWS.csv"
  PARTITION_TABLE_BIN="images/partition_table_8MB.bin"
else
  PARTITIONS_FILE="partitions/partitions_VALVES.csv"
  PARTITION_TABLE_BIN="images/partition_table_4MB.bin"
fi

WEBPAGE_DIR="webapp" # Local directory containing webpage files

echo "Debug: PCB_TYPE is set to: $PCB_TYPE"

# Reconfigure the project with the correct PCB_TYPE
echo "Reconfiguring project for ${PCB_TYPE}..."
idf.py reconfigure

# Check if binary exists
if [ ! -f "images/${BINARY_NAME}" ]; then
  echo "Error: ${BINARY_NAME} not found in images directory."
  exit 1
fi

# # For newer ESP chips, it's often safer to erase the entire flash before flashing
# if [ "$ESP_CHIP" == "esp32s3" ] || [ "$ESP_CHIP" == "esp32c3" ]; then
#   echo "Erasing entire flash for $ESP_CHIP to ensure clean state..."
#   python $IDF_PATH/components/esptool_py/esptool/esptool.py \
#     --chip $ESP_CHIP \
#     --port $ESPPORT \
#     --baud 921600 \
#     erase_flash
#
#   if [ $? -ne 0 ]; then
#     echo "Error: Failed to erase flash"
#     exit 1
#   fi
#
#   echo "Flash erased successfully"
# fi

# Now use idf.py flash with the auto-detected port
echo "Flashing with idf.py using verified project configuration..."
idf.py -p $ESPPORT flash

if [ $? -ne 0 ]; then
  echo "Error: idf.py flash failed."
  exit 1
fi

echo "Firmware flashed successfully with idf.py"

# For web app, we still need custom handling
if [ "$INCLUDE_WEB_APP" = "web_app" ]; then
  if [ ! -f "${PARTITIONS_FILE}" ]; then
    echo "Error: ${PARTITIONS_FILE} not found."
    exit 1
  fi

  if [ ! -d "${WEBPAGE_DIR}" ]; then
    echo "Error: Webpage directory not found. Please ensure the ${WEBPAGE_DIR} directory exists and contains the webpage files."
    exit 1
  fi

  # Function to parse the entire partition table
  parse_partition_table() {
    awk -F, '
        NR > 1 {
            gsub(/^[ \t]+|[ \t]+$/, "", $1)  # Trim whitespace from name
            gsub(/^[ \t]+|[ \t]+$/, "", $2)  # Trim whitespace from type
            gsub(/^[ \t]+|[ \t]+$/, "", $3)  # Trim whitespace from subtype
            gsub(/^[ \t]+|[ \t]+$/, "", $4)  # Trim whitespace from offset
            gsub(/^[ \t]+|[ \t]+$/, "", $5)  # Trim whitespace from size
            print $1 "," $2 "," $3 "," $4 "," $5
        }
        ' "${PARTITIONS_FILE}"
  }

  # Parse the partition table
  PARTITION_INFO=$(parse_partition_table)

  # Function to get partition information
  get_partition_info() {
    local partition_name=$1
    echo "$PARTITION_INFO" | grep "^${partition_name}," | head -n 1
  }

  # Function to convert size to bytes
  convert_size_to_bytes() {
    local size=$1
    if [[ $size =~ K$ ]]; then
      echo $((${size%K} * 1024))
    else
      echo $size
    fi
  }

  # Get storage partition info
  STORAGE_PARTITION=$(get_partition_info "storage")
  STORAGE_OFFSET=$(echo "$STORAGE_PARTITION" | cut -d',' -f4)
  STORAGE_SIZE=$(echo "$STORAGE_PARTITION" | cut -d',' -f5)

  # Convert storage size to bytes and calculate aligned size
  STORAGE_SIZE_BYTES=$(convert_size_to_bytes "$STORAGE_SIZE")
  SECTORS=$(((STORAGE_SIZE_BYTES + 4095) / 4096))
  ALIGNED_SIZE_BYTES=$((SECTORS * 4096))

  echo "Storage partition found at offset ${STORAGE_OFFSET} with size ${STORAGE_SIZE}"

  # Format the storage partition
  echo "Formatting storage partition for web app..."
  python $IDF_PATH/components/esptool_py/esptool/esptool.py \
    --chip $ESP_CHIP \
    --port $ESPPORT \
    --baud 921600 \
    erase_region ${STORAGE_OFFSET} ${ALIGNED_SIZE_BYTES}

  if [ $? -ne 0 ]; then
    echo "Error: Failed to format storage partition for web app."
    exit 1
  fi

  # Create SPIFFS image from local webpage directory
  echo "Creating SPIFFS image for web app..."
  python $IDF_PATH/components/spiffs/spiffsgen.py ${ALIGNED_SIZE_BYTES} ${WEBPAGE_DIR} images/spiffs.bin

  if [ $? -ne 0 ]; then
    echo "Error: Failed to create SPIFFS image for web app."
    exit 1
  fi

  # Flash the web app
  echo "Flashing web app to storage partition..."
  python $IDF_PATH/components/esptool_py/esptool/esptool.py \
    --chip $ESP_CHIP \
    --port $ESPPORT \
    --baud 921600 \
    write_flash ${STORAGE_OFFSET} images/spiffs.bin

  if [ $? -ne 0 ]; then
    echo "Error: Failed to flash web app."
    exit 1
  fi

  echo "Web app flashed successfully"
fi

echo "Starting monitor..."
idf.py -p $ESPPORT monitor
