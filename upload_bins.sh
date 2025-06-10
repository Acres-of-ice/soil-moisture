#!/bin/bash

# Function to show script usage
show_usage() {
  echo "Usage: $0 [options]"
  echo "Options:"
  echo "  -a, --all        Upload all binary files"
  echo "  -c, --conductor  Upload only Master files (default)"
  echo "  -h, --help       Show this help message"
  echo
  echo "Example:"
  echo "  $0              # Uploads only Master files"
  echo "  $0 --all        # Uploads all binary files"
}

# Function to upload files to AWS S3
upload_to_s3() {
  local upload_all=$1
  echo "Initializing upload to AWS S3..."

  # Configuration
  S3_BUCKET="pcb-bins"

  # Check if images directory exists
  if [ ! -d "images" ]; then
    echo -e "\033[1;31mError: 'images' directory not found.\033[0m"
    echo "Please run this script from the project root directory."
    exit 1
  fi

  # Initialize upload message
  if [ "$upload_all" = true ]; then
    echo "Uploading all binary files..."
    # Sync all binary files
    aws s3 sync images "s3://$S3_BUCKET" --exclude "*" --include "*.bin"
  else
    echo "Uploading Master files only..."
    # Sync Master binaries
    echo "Syncing Master binary..."
    aws s3 sync images "s3://$S3_BUCKET" --exclude "*" --include "*MASTER.bin"
  fi

  # Check upload status
  if [ $? -eq 0 ]; then
    echo -e "\n\033[1;32mUpload completed successfully!\033[0m"
    echo -e "\nUploaded files:"

    # List uploaded files
    if [ "$upload_all" = true ]; then
      echo -e "\033[1;34mAll binary files in images directory:\033[0m"
      ls -1 images/*.bin
    else
      echo -e "\033[1;34mMaster files:\033[0m"
      ls -1 images/*_MASTER.bin 2>/dev/null
    fi
  else
    echo -e "\033[1;31mError: Upload failed. Please check your AWS configuration and try again.\033[0m"
    exit 1
  fi
}

# Parse command line arguments
upload_all=false

while [[ $# -gt 0 ]]; do
  case $1 in
  -a | --all)
    upload_all=true
    shift
    ;;
  -c | --conductor)
    upload_all=false
    shift
    ;;
  -h | --help)
    show_usage
    exit 0
    ;;
  *)
    echo "Unknown option: $1"
    show_usage
    exit 1
    ;;
  esac
done

# Execute upload
upload_to_s3 $upload_all
