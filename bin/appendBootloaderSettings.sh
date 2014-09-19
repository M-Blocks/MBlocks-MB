#!/bin/bash
(set -o igncr) 2>/dev/null && set -o igncr; # this comment is needed

DEBUG=0

# This is the location where the bootloader settings are stored.  It 
# corresponds to the beginning of the last 1024-byte sector of the 256KB
# flash memory. 
BL_SETTINGS_LOC=0x3FC00

# Enable echo if DEBUG=1
if [ $DEBUG == "1" ]; then
	set -x #echo on
fi

# Verify input argument exists
if [ $# -ne 2 ]; then
	echo "$(basename $0) expects two argument: the hex file to which the bootloader settings should be appended and the name of the resulting output file."
	exit -1
fi

# Verify that the file name passed as the first argument actually exists
if [ ! -f $1 ]; then
	echo "$1 does not exist"
	exit -1
fi

# Begin by making copy of the input file whose new name is the output file name
cp -f ${1} ${2}

# Exclude the bytes we need to (over-)write from the file.  The -fill filter does
# not work if there are already bytes the locations that it is trying to fill.
srec_cat ${2} -Intel \
	-Exclude $((BL_SETTINGS_LOC)) $(expr $((BL_SETTINGS_LOC)) + 4) \
    -Output ${2} -Intel

# Write 0x01 to mark the application as valid
srec_cat ${2} -Intel \
    -fill 0x01 $((BL_SETTINGS_LOC)) $(expr $((BL_SETTINGS_LOC)) + 1) \
    -Output ${2} -Intel

# Write a checksum of 0x0000 to instruct the bootloader that the application's
# checksum does not need to be verified.
srec_cat ${2} -Intel \
    -fill 0x00 $(expr $((BL_SETTINGS_LOC)) + 2) $(expr $((BL_SETTINGS_LOC)) + 4) \
    -Output ${2} -Intel
	
# Reformat the hex file to match original format
if [ $DEBUG == "1" ]; then
	cp	${2} ${2}.prereformat
fi

srec_cat ${2} -Intel \
    -o ${2} -Intel --line_length=44

exit 0
