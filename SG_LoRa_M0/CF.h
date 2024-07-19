/*
 * ======================================================================================================================
 *  CF.h - Configuration File - CONFIG.TXT
 * ======================================================================================================================
 */

/* 
 * ======================================================================================================================
#
# CONFIG.TXT
#
# Line Length is limited to 63 characters
#12345678901234567890123456789012345678901234567890123456789012

# Private Key - 128 bits (16 bytes of ASCII characters)
aes_pkey=10FE2D3C4B5A6978

# Initialization Vector must be and always will be 128 bits (16 bytes.)
# The real iv is actually myiv repeated twice
# 1234567 -> 0x12D687 = 0x00 0x12 0xD6 0x87 0x00 0x12 0xD6 0x87 
aes_myiv=1234567

# This unit's LoRa ID for Receiving and Sending Messages
# Also Chords Site instrument_id
lora_unitid=2

# LoRa Address who we are sending to aka LoRa 3D-PAWS
lora_gwid=1

# You can set transmitter power from 5 to 23 dBm, default is 13
lora_txpower=23

# Valid entries are 433, 866, 915
lora_freq=915

# Distance sensor type 0 = 5m (default), 1 = 10m
ds_type=0
 * ======================================================================================================================
 */

/*
 * ======================================================================================================================
 *  Define Global Configuration File Variables
 * ======================================================================================================================
 */
char *cf_aes_pkey=NULL;
long cf_aes_myiv=0;
int cf_lora_unitid=2;
int cf_lora_gwid=1;
int cf_lora_txpower=13;
int cf_lora_freq=915;
int cf_ds_type=0;
