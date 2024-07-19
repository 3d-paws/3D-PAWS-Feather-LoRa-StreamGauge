/*
 * ======================================================================================================================
 * StationMonitor() - When Jumper set Display StationMonitor()
 * ======================================================================================================================
 */

/*
 * ======================================================================================================================
 * StationMonitor() - On OLED display station information
 * ======================================================================================================================
 */
void StationMonitor() {
  static int b = 0;
  static int p = 0;  // use to loop through each probe 0,1,2
  int r, c, len;
  
  float bmx_pressure = 0.0;
  float bmx_temp = 0.0;
  float bmx_humid = 0.0;
  char bmx1[16];
  char bmx2[16];
  char Buffer16Bytes[16];

  float batt = vbat_get();

  OLED_ClearDisplayBuffer();

  // =================================================================
  // Line 0 of OLED
  // =================================================================
  rtc_timestamp();
  len = (strlen (timestamp) > 21) ? 21 : strlen (timestamp);
  for (c=0; c<=len; c++) oled_lines [0][c] = *(timestamp+c);
  Serial_write (timestamp);

  // =================================================================
  // Line 1 of OLED  [BMX P BMX P]
  // =================================================================
  if (BMX_1_exists) {
    switch (BMX_1_chip_id) {
       case BMP280_CHIP_ID :
         bmx_pressure = bmp1.readPressure()/100.0F;           // bmxp1
         bmx_temp = bmp1.readTemperature();                   // bmxt1
         bmx_humid = 0.0;
       break;
       case BME280_BMP390_CHIP_ID :
         if (BMX_1_chip_id == BME280_BMP390_CHIP_ID) {
           bmx_pressure = bme1.readPressure()/100.0F;         // bmxp1
           bmx_temp = bme1.readTemperature();                 // bmxt1
           bmx_humid = bme1.readHumidity();                   // bmxh1 
         }
         else { // BMP390
           bmx_pressure = bm31.readPressure()/100.0F;
           bmx_temp = bm31.readTemperature();
           bmx_humid = 0.0;
         }
       break;
       case BMP388_CHIP_ID :
          bmx_pressure = bm31.readPressure()/100.0F;
          bmx_temp = bm31.readTemperature();
          bmx_humid = 0.0;
       break;
       default:
          Output ("BMX:WTF"); // This should not happen.
       break;
    }

    // Raw values no quality checks
    switch (b) {
      case 0: sprintf (bmx1, "P1:%d.%02d", (int)bmx_pressure, (int)(bmx_pressure*100)%100); break;
      case 1: sprintf (bmx1, "T1:%d.%02d", (int)bmx_temp, (int)(bmx_temp*100)%100); break;
      case 2: sprintf (bmx1, "H1:%d.%02d", (int)bmx_humid, (int)(bmx_humid*100)%100); break;
    }
  }
  else {
    sprintf (bmx1, "BMX1:NF");
  }

  if (BMX_2_exists) {
    switch (BMX_2_chip_id) {
       case BMP280_CHIP_ID :
         bmx_pressure = bmp2.readPressure()/100.0F;           // bmxp1
         bmx_temp = bmp2.readTemperature();                   // bmxt1
         bmx_humid = 0.0;
       break;
       case BME280_BMP390_CHIP_ID :
         if (BMX_2_chip_id == BME280_BMP390_CHIP_ID) {
           bmx_pressure = bme2.readPressure()/100.0F;         // bmxp1
           bmx_temp = bme2.readTemperature();                 // bmxt1
           bmx_humid = bme2.readHumidity();                   // bmxh1 
         }
         else { // BMP390
           bmx_pressure = bm32.readPressure()/100.0F;
           bmx_temp = bm32.readTemperature();
           bmx_humid = 0.0;
         }
       break;
       case BMP388_CHIP_ID :
          bmx_pressure = bm32.readPressure()/100.0F;
          bmx_temp = bm32.readTemperature();
          bmx_humid = 0.0;
       break;
       default:
          Output ("BMX:WTF"); // This should not happen.
       break;
    }

    // Raw values no quality checks
    switch (b) {
      case 0: sprintf (bmx2, "P2:%d.%02d", (int)bmx_pressure, (int)(bmx_pressure*100)%100); break;
      case 1: sprintf (bmx2, "T2:%d.%02d", (int)bmx_temp, (int)(bmx_temp*100)%100); break;
      case 2: sprintf (bmx2, "H2:%d.%02d", (int)bmx_humid, (int)(bmx_humid*100)%100); break;
    }
  }
  else {
    sprintf (bmx2, "BMX2:NF");
  }
  b = (++b) % 3;

  sprintf (Buffer32Bytes, "%s %s", bmx1, bmx2);
  
  len = (strlen (Buffer32Bytes) > 21) ? 21 : strlen (Buffer32Bytes);
  for (c=0; c<=len; c++) oled_lines [1][c] = *(Buffer32Bytes+c);
  Serial_write (Buffer32Bytes);


  // =================================================================
  // Line 2 of OLED
  // =================================================================
  sprintf (Buffer32Bytes, "");
  
  len = (strlen (Buffer32Bytes) > 21) ? 21 : strlen (Buffer32Bytes);
  for (c=0; c<=len; c++) oled_lines [2][c] = *(Buffer32Bytes+c);
  Serial_write (Buffer32Bytes);
  
  // =================================================================
  // Line 3 of OLED
  // =================================================================
  float sg;
  if (cf_ds_type) {  // 0 = 5m, 1 = 10m
    sg = analogRead(SGAUGE_PIN)*5;
  }
  else {
    sg = analogRead(SGAUGE_PIN)*10;
  }
  sprintf (Buffer32Bytes, "SG:%d.%02d %d.%02d %04X", 
    (int)sg, (int)(sg*100)%100,
    (int)batt, (int)(batt*100)%100,
    SystemStatusBits); 

  len = (strlen (Buffer32Bytes) > 21) ? 21 : strlen (Buffer32Bytes);
  for (c=0; c<=len; c++) oled_lines [3][c] = *(Buffer32Bytes+c);
  Serial_write (Buffer32Bytes);

  OLED_update();
}
