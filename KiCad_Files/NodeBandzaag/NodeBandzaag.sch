EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
Title "Node Bandzaag"
Date "2021-12-14"
Rev "V0.3"
Comp "MakerSpace Leiden"
Comment1 "Getekend door: Hans Beerman"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector_Generic:Conn_01x10 J4
U 1 1 5E1FBCBC
P 6050 1850
F 0 "J4" H 6000 1250 50  0000 L CNN
F 1 "EXT1" H 5950 2400 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x10_P2.54mm_Vertical" H 6050 1850 50  0001 C CNN
F 3 "~" H 6050 1850 50  0001 C CNN
	1    6050 1850
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x10 J7
U 1 1 5E1FBCC2
P 6750 1850
F 0 "J7" H 6700 1250 50  0000 L CNN
F 1 "EXT2" H 6650 2400 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x10_P2.54mm_Vertical" H 6750 1850 50  0001 C CNN
F 3 "~" H 6750 1850 50  0001 C CNN
	1    6750 1850
	-1   0    0    -1  
$EndComp
Text GLabel 5650 1750 0    50   Input ~ 0
ESP_EN
$Comp
L power:+3.3V #PWR03
U 1 1 5E1FBCC9
P 5700 1550
F 0 "#PWR03" H 5700 1400 50  0001 C CNN
F 1 "+3.3V" V 5715 1678 50  0000 L CNN
F 2 "" H 5700 1550 50  0001 C CNN
F 3 "" H 5700 1550 50  0001 C CNN
	1    5700 1550
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR02
U 1 1 5E1FBCCF
P 5700 1450
F 0 "#PWR02" H 5700 1300 50  0001 C CNN
F 1 "+5V" V 5715 1578 50  0000 L CNN
F 2 "" H 5700 1450 50  0001 C CNN
F 3 "" H 5700 1450 50  0001 C CNN
	1    5700 1450
	0    -1   -1   0   
$EndComp
Text GLabel 5650 2050 0    50   BiDi ~ 0
GPIO2_HS2_DATA0
Text GLabel 5650 1950 0    50   BiDi ~ 0
GPIO1_U0TXD
Wire Wire Line
	5700 1550 5850 1550
Wire Wire Line
	5700 1650 5850 1650
Wire Wire Line
	5650 1750 5850 1750
Wire Wire Line
	5650 1950 5850 1950
Wire Wire Line
	5650 2050 5850 2050
Text GLabel 5650 1850 0    50   BiDi ~ 0
GPIO0
Wire Wire Line
	5650 1850 5850 1850
Text GLabel 5650 2150 0    50   BiDi ~ 0
GPIO3_U0RXD
Wire Wire Line
	5650 2150 5850 2150
Text GLabel 5650 2250 0    50   BiDi ~ 0
GPIO4_U1TXD
Wire Wire Line
	5650 2250 5850 2250
Text GLabel 5650 2350 0    50   BiDi ~ 0
GPIO5_SPI_C5
Wire Wire Line
	5650 2350 5850 2350
Text Notes 5900 1200 0    59   ~ 12
ESP32-PoE connectors
Text GLabel 7200 1550 2    50   Input ~ 0
GPI36_U1RXD
Text GLabel 7200 1650 2    50   Input ~ 0
GPI35
Text GLabel 7200 1850 2    50   BiDi ~ 0
GPIO33
Text GLabel 7200 1950 2    50   BiDi ~ 0
GPIO32
Text GLabel 7200 2050 2    50   BiDi ~ 0
GPIO16_I2C-SCL
Text GLabel 7200 2150 2    50   BiDi ~ 0
GPIO15_HS2_CMD
Text GLabel 7200 2250 2    50   BiDi ~ 0
GPIO14_HS2_CLK
Text GLabel 7200 2350 2    50   BiDi ~ 0
GPIO13_I2C-SDA
Text GLabel 7200 1750 2    50   Input ~ 0
GPI34_BUT1
Wire Wire Line
	6950 1450 7200 1450
Wire Wire Line
	6950 1550 7200 1550
Wire Wire Line
	6950 1650 7200 1650
Wire Wire Line
	6950 1750 7200 1750
Wire Wire Line
	6950 1850 7200 1850
Wire Wire Line
	6950 1950 7200 1950
Wire Wire Line
	6950 2050 7200 2050
Wire Wire Line
	6950 2150 7200 2150
Wire Wire Line
	6950 2250 7200 2250
Wire Wire Line
	6950 2350 7200 2350
Text Notes 5950 3800 0    59   ~ 12
Rfid
$Comp
L power:GND #PWR04
U 1 1 5E2A9C2D
P 5700 1650
F 0 "#PWR04" H 5700 1400 50  0001 C CNN
F 1 "GND" V 5705 1522 50  0000 R CNN
F 2 "" H 5700 1650 50  0001 C CNN
F 3 "" H 5700 1650 50  0001 C CNN
	1    5700 1650
	0    1    1    0   
$EndComp
$Comp
L Connector:Screw_Terminal_01x03 J13
U 1 1 5E576D88
P 12125 2150
F 0 "J13" H 12075 2400 50  0000 L CNN
F 1 "Relay 1 Output" H 11975 1900 50  0000 L CNN
F 2 "Connector_Phoenix_MSTB:PhoenixContact_MSTB_2,5_3-GF-5,08_1x03_P5.08mm_Horizontal_ThreadedFlange" H 12125 2150 50  0001 C CNN
F 3 "~" H 12125 2150 50  0001 C CNN
	1    12125 2150
	1    0    0    1   
$EndComp
Wire Wire Line
	11225 1850 11225 1750
Wire Wire Line
	11225 1750 11925 1750
Wire Wire Line
	11925 1750 11925 2050
$Comp
L power:+5V #PWR026
U 1 1 5E59AF4F
P 10325 1800
F 0 "#PWR026" H 10325 1650 50  0001 C CNN
F 1 "+5V" H 10225 2000 50  0000 L CNN
F 2 "" H 10325 1800 50  0001 C CNN
F 3 "" H 10325 1800 50  0001 C CNN
	1    10325 1800
	1    0    0    -1  
$EndComp
$Comp
L PC814:PC814 OK1
U 1 1 5E5E62CA
P 9875 7150
F 0 "OK1" H 9925 7520 50  0000 C CNN
F 1 "PC814" H 9925 7429 50  0000 C CNN
F 2 "Package_DIP:DIP-4_W7.62mm" H 9875 7150 50  0001 L BNN
F 3 "" H 9875 7150 50  0001 L BNN
F 4 "unknown" H 9875 7150 50  0001 L BNN "Field4"
F 5 "9707662" H 9875 7150 50  0001 L BNN "Field5"
F 6 "" H 9875 7150 50  0001 L BNN "Field6"
	1    9875 7150
	-1   0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J10
U 1 1 5E5F8709
P 11175 7100
F 0 "J10" H 11125 7250 50  0000 L CNN
F 1 "Opto Coupler 1 in" H 11325 7250 50  0000 L CNN
F 2 "Connector_Phoenix_MSTB:PhoenixContact_MSTB_2,5_2-GF-5,08_1x02_P5.08mm_Horizontal_ThreadedFlange" H 11175 7100 50  0001 C CNN
F 3 "~" H 11175 7100 50  0001 C CNN
	1    11175 7100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R14
U 1 1 5E5F9735
P 10625 7250
F 0 "R14" V 10725 7250 50  0000 C CNN
F 1 "220k" V 10625 7250 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0516_L15.5mm_D5.0mm_P20.32mm_Horizontal" V 10555 7250 50  0001 C CNN
F 3 "~" H 10625 7250 50  0001 C CNN
	1    10625 7250
	0    1    1    0   
$EndComp
Wire Wire Line
	10775 7250 10875 7250
Wire Wire Line
	10875 7250 10875 7200
Wire Wire Line
	10875 7200 10975 7200
Wire Wire Line
	10775 7050 10875 7050
Wire Wire Line
	10875 7050 10875 7100
Wire Wire Line
	10875 7100 10975 7100
$Comp
L power:GND #PWR021
U 1 1 5E608E4F
P 9375 7500
F 0 "#PWR021" H 9375 7250 50  0001 C CNN
F 1 "GND" H 9380 7327 50  0000 C CNN
F 2 "" H 9375 7500 50  0001 C CNN
F 3 "" H 9375 7500 50  0001 C CNN
	1    9375 7500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9375 7250 9375 7500
Text Notes 11075 6850 0    59   ~ 12
Opto Coupler input 1
Text Notes 11975 1650 0    59   ~ 12
Relay 1 output
$Comp
L Device:LED D3
U 1 1 5E5BC9B9
P 10175 2500
F 0 "D3" V 10214 2383 50  0000 R CNN
F 1 "LED" V 10123 2383 50  0000 R CNN
F 2 "LED_THT:LED_D3.0mm" H 10175 2500 50  0001 C CNN
F 3 "~" H 10175 2500 50  0001 C CNN
	1    10175 2500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	10175 2300 10175 2350
Wire Wire Line
	10175 2000 10175 1850
Text Notes 12225 2085 0    49   ~ 0
NC
Text Notes 12225 2275 0    49   ~ 0
NO
Text Notes 12225 2175 0    49   ~ 0
Common
Text Notes 11280 7185 0    49   ~ 0
Default 230V AC
$Comp
L Diode:1N4004 D5
U 1 1 5E593BB3
P 10525 2150
F 0 "D5" H 10625 2250 50  0000 L CNN
F 1 "1N4004" H 10275 2250 50  0000 L CNN
F 2 "Diode_THT:D_DO-41_SOD81_P10.16mm_Horizontal" H 10525 1975 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88503/1n4001.pdf" H 10525 2150 50  0001 C CNN
	1    10525 2150
	0    1    1    0   
$EndComp
Wire Wire Line
	11325 2450 11725 2450
Wire Wire Line
	11725 2450 11725 2150
Wire Wire Line
	11725 2150 11925 2150
Wire Wire Line
	11425 1850 11825 1850
Wire Wire Line
	11825 1850 11825 2250
Wire Wire Line
	11825 2250 11925 2250
$Comp
L Relay:SANYOU_SRD_Form_C K1
U 1 1 5E706EDD
P 11125 2150
F 0 "K1" H 11075 1800 50  0000 L CNN
F 1 "SANYOU_SRD_Form_C" H 10725 2650 50  0000 L CNN
F 2 "Relay_THT:Relay_SPDT_SANYOU_SRD_Series_Form_C" H 11575 2100 50  0001 L CNN
F 3 "http://www.sanyourelay.ca/public/products/pdf/SRD.pdf" H 11125 2150 50  0001 C CNN
	1    11125 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	10525 2000 10525 1850
Connection ~ 10525 1850
Wire Wire Line
	10525 1850 10925 1850
$Comp
L Device:R R13
U 1 1 5E5F9EE9
P 10625 7050
F 0 "R13" V 10725 7050 50  0000 C CNN
F 1 "220k" V 10625 7050 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0516_L15.5mm_D5.0mm_P20.32mm_Horizontal" V 10555 7050 50  0001 C CNN
F 3 "~" H 10625 7050 50  0001 C CNN
	1    10625 7050
	0    1    1    0   
$EndComp
Wire Wire Line
	10275 7050 10475 7050
$Comp
L Device:R R11
U 1 1 5E5D8FE6
P 10175 2150
F 0 "R11" H 10245 2196 50  0000 L CNN
F 1 "2k2" V 10175 2100 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 10105 2150 50  0001 C CNN
F 3 "~" H 10175 2150 50  0001 C CNN
	1    10175 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	10175 2800 10175 2650
Wire Wire Line
	9875 2800 10175 2800
$Comp
L Interface_Expansion:MCP23017_SP U1
U 1 1 5FC24DD6
P 6175 8975
F 0 "U1" H 5825 10150 50  0000 C CNN
F 1 "MCP23017_SP" H 5800 10025 50  0000 C CNN
F 2 "Package_DIP:DIP-28_W7.62mm" H 6375 7975 50  0001 L CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20001952C.pdf" H 6375 7875 50  0001 L CNN
	1    6175 8975
	1    0    0    -1  
$EndComp
Wire Wire Line
	10325 1800 10325 1850
Wire Wire Line
	10175 1850 10325 1850
Connection ~ 10325 1850
Wire Wire Line
	10325 1850 10525 1850
Wire Wire Line
	10175 2800 10525 2800
Wire Wire Line
	10925 2800 10925 2450
Connection ~ 10175 2800
Wire Wire Line
	10525 2300 10525 2800
Connection ~ 10525 2800
Wire Wire Line
	10525 2800 10925 2800
$Comp
L Connector_Generic:Conn_01x02 J8
U 1 1 5E6404FC
P 11125 2950
F 0 "J8" H 11075 3050 50  0000 L CNN
F 1 "Force output Relay 1 on" H 11050 2675 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 11125 2950 50  0001 C CNN
F 3 "~" H 11125 2950 50  0001 C CNN
	1    11125 2950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 5E68BC49
P 9375 3000
F 0 "R9" V 9275 2950 50  0000 L CNN
F 1 "1k" V 9375 2950 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 9305 3000 50  0001 C CNN
F 3 "~" H 9375 3000 50  0001 C CNN
	1    9375 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	9525 3000 9575 3000
Wire Wire Line
	9875 3300 9875 3200
$Comp
L power:GND #PWR024
U 1 1 5E67C879
P 9875 3300
F 0 "#PWR024" H 9875 3050 50  0001 C CNN
F 1 "GND" H 9880 3127 50  0000 C CNN
F 2 "" H 9875 3300 50  0001 C CNN
F 3 "" H 9875 3300 50  0001 C CNN
	1    9875 3300
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:BC547 Q1
U 1 1 5E6740A8
P 9775 3000
F 0 "Q1" H 9966 3046 50  0000 L CNN
F 1 "BC547" H 9966 2955 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 9975 2925 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/BC/BC547.pdf" H 9775 3000 50  0001 L CNN
	1    9775 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	10925 3300 10925 3050
$Comp
L power:GND #PWR028
U 1 1 5E6923E9
P 10925 3300
F 0 "#PWR028" H 10925 3050 50  0001 C CNN
F 1 "GND" H 10930 3127 50  0000 C CNN
F 2 "" H 10925 3300 50  0001 C CNN
F 3 "" H 10925 3300 50  0001 C CNN
	1    10925 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	10925 2950 10925 2800
Connection ~ 10925 2800
$Comp
L Jumper:SolderJumper_2_Open JP4
U 1 1 6003E69B
P 11725 2750
F 0 "JP4" V 11679 2818 50  0000 L CNN
F 1 "230VAC-L" V 11770 2818 50  0000 L CNN
F 2 "Footprints:SolderJumper-2_P3mm_Open_RoundedPad" H 11725 2750 50  0001 C CNN
F 3 "~" H 11725 2750 50  0001 C CNN
	1    11725 2750
	0    1    1    0   
$EndComp
Wire Wire Line
	11725 2600 11725 2450
Connection ~ 11725 2450
Text GLabel 11900 3100 2    50   Input ~ 0
230VAC-L
Wire Wire Line
	11900 3100 11725 3100
Wire Wire Line
	11725 3100 11725 2900
Text GLabel 7150 9075 2    50   Input ~ 0
Switch1
Text GLabel 7150 9175 2    50   Input ~ 0
Switch2
Text GLabel 7150 9275 2    50   Input ~ 0
Switch3
Wire Wire Line
	6875 9075 7150 9075
Wire Wire Line
	7150 9175 6875 9175
Wire Wire Line
	6875 9275 7150 9275
Text GLabel 9000 3000 0    50   Input ~ 0
Relay1
Wire Wire Line
	9000 3000 9225 3000
Text GLabel 7175 8175 2    50   Output ~ 0
Relay1
Wire Wire Line
	6875 8175 7175 8175
Text GLabel 7175 8275 2    50   Output ~ 0
Relay2
Wire Wire Line
	6875 8275 7175 8275
Text GLabel 7175 8475 2    50   Output ~ 0
FET1
Wire Wire Line
	6875 8475 7175 8475
Text GLabel 7175 8575 2    50   Output ~ 0
FET2
Wire Wire Line
	6875 8575 7175 8575
$Comp
L power:GND #PWR06
U 1 1 5FD9011F
P 6175 10200
F 0 "#PWR06" H 6175 9950 50  0001 C CNN
F 1 "GND" H 6180 10027 50  0000 C CNN
F 2 "" H 6175 10200 50  0001 C CNN
F 3 "" H 6175 10200 50  0001 C CNN
	1    6175 10200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6175 10200 6175 10125
$Comp
L power:+3.3V #PWR05
U 1 1 5FDAB66E
P 6175 7750
F 0 "#PWR05" H 6175 7600 50  0001 C CNN
F 1 "+3.3V" H 6190 7878 50  0000 L CNN
F 2 "" H 6175 7750 50  0001 C CNN
F 3 "" H 6175 7750 50  0001 C CNN
	1    6175 7750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6175 7875 6175 7750
Text GLabel 8500 7050 0    50   Output ~ 0
GPIO32
$Comp
L power:PWR_FLAG #FLG01
U 1 1 5FEA2B74
P 14575 900
F 0 "#FLG01" H 14575 975 50  0001 C CNN
F 1 "PWR_FLAG" H 14575 1075 50  0000 C CNN
F 2 "" H 14575 900 50  0001 C CNN
F 3 "~" H 14575 900 50  0001 C CNN
	1    14575 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	14175 900  14175 1000
Wire Wire Line
	14175 1000 14575 1000
Wire Wire Line
	14575 1000 14575 900 
$Comp
L power:+3.3V #PWR057
U 1 1 5FEB4FC6
P 14175 900
F 0 "#PWR057" H 14175 750 50  0001 C CNN
F 1 "+3.3V" H 14050 1075 50  0000 L CNN
F 2 "" H 14175 900 50  0001 C CNN
F 3 "" H 14175 900 50  0001 C CNN
	1    14175 900 
	1    0    0    -1  
$EndComp
Text GLabel 6825 3175 2    50   BiDi ~ 0
GPIO13_I2C-SDA
Text GLabel 6825 3275 2    50   BiDi ~ 0
GPIO16_I2C-SCL
$Comp
L power:+3.3V #PWR09
U 1 1 5FF4729E
P 8250 2650
F 0 "#PWR09" H 8250 2500 50  0001 C CNN
F 1 "+3.3V" V 8265 2778 50  0000 L CNN
F 2 "" H 8250 2650 50  0001 C CNN
F 3 "" H 8250 2650 50  0001 C CNN
	1    8250 2650
	-1   0    0    -1  
$EndComp
Text GLabel 5350 8175 0    50   BiDi ~ 0
GPIO13_I2C-SDA
Wire Wire Line
	5350 8175 5475 8175
Text GLabel 5350 8275 0    50   BiDi ~ 0
GPIO16_I2C-SCL
Wire Wire Line
	5350 8275 5475 8275
$Comp
L Device:R R2
U 1 1 6001E5EF
P 6300 2925
F 0 "R2" H 6370 2971 50  0000 L CNN
F 1 "4k7" V 6300 2875 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6230 2925 50  0001 C CNN
F 3 "~" H 6300 2925 50  0001 C CNN
	1    6300 2925
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 6001F2D7
P 6675 2925
F 0 "R4" H 6745 2971 50  0000 L CNN
F 1 "4k7" V 6675 2875 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6605 2925 50  0001 C CNN
F 3 "~" H 6675 2925 50  0001 C CNN
	1    6675 2925
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 3075 6300 3175
Wire Wire Line
	6300 3175 6825 3175
Wire Wire Line
	6300 2775 6500 2775
Connection ~ 6500 2775
Wire Wire Line
	6500 2775 6675 2775
Wire Wire Line
	6675 3075 6675 3275
Wire Wire Line
	6675 3275 6825 3275
Wire Wire Line
	5475 10125 6175 10125
Wire Wire Line
	5475 9775 5475 10125
Connection ~ 6175 10125
Wire Wire Line
	6175 10125 6175 10075
Wire Wire Line
	5475 9575 5475 9675
Wire Wire Line
	5475 9675 5475 9775
Connection ~ 5475 9675
Connection ~ 5475 9775
$Comp
L power:+3.3V #PWR01
U 1 1 600FDF2F
P 5000 8975
F 0 "#PWR01" H 5000 8825 50  0001 C CNN
F 1 "+3.3V" H 5015 9103 50  0000 L CNN
F 2 "" H 5000 8975 50  0001 C CNN
F 3 "" H 5000 8975 50  0001 C CNN
	1    5000 8975
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 8975 5000 9075
Wire Wire Line
	5000 9075 5475 9075
$Comp
L Connector:Screw_Terminal_01x04 J6
U 1 1 5FB1C920
P 6050 4175
F 0 "J6" H 6000 4425 50  0000 L CNN
F 1 "RFIDReader" H 6200 4425 50  0000 L CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MPT-0,5-4-2.54_1x04_P2.54mm_Horizontal" H 6050 4175 50  0001 C CNN
F 3 "~" H 6050 4175 50  0001 C CNN
	1    6050 4175
	-1   0    0    -1  
$EndComp
Text GLabel 6825 4175 2    50   BiDi ~ 0
GPIO13_I2C-SDA
Text GLabel 6825 4075 2    50   BiDi ~ 0
GPIO16_I2C-SCL
$Comp
L power:GND #PWR013
U 1 1 5FB1C928
P 7175 4375
F 0 "#PWR013" H 7175 4125 50  0001 C CNN
F 1 "GND" H 7180 4202 50  0000 C CNN
F 2 "" H 7175 4375 50  0001 C CNN
F 3 "" H 7175 4375 50  0001 C CNN
	1    7175 4375
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 4375 7175 4375
Wire Wire Line
	6250 4075 6825 4075
Wire Wire Line
	6250 4275 6500 4275
Wire Wire Line
	6250 4175 6825 4175
Text Notes 5950 4200 2    49   ~ 0
SDA
Text Notes 5950 4100 2    49   ~ 0
SCL
Text Notes 5950 4300 2    49   ~ 0
+3V3
Text Notes 5950 4400 2    49   ~ 0
GND
Text GLabel 7200 1450 2    50   Input ~ 0
GPI39
$Comp
L Device:R R5
U 1 1 5FAE9D46
P 7500 5975
F 0 "R5" H 7570 6021 50  0000 L CNN
F 1 "22k" V 7500 5925 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7430 5975 50  0001 C CNN
F 3 "~" H 7500 5975 50  0001 C CNN
	1    7500 5975
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 5FAEA41A
P 7500 6500
F 0 "R6" H 7570 6546 50  0000 L CNN
F 1 "22k" V 7500 6450 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7430 6500 50  0001 C CNN
F 3 "~" H 7500 6500 50  0001 C CNN
	1    7500 6500
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C1
U 1 1 5FAF0BF8
P 6975 6500
F 0 "C1" H 7093 6546 50  0000 L CNN
F 1 "10uF" H 7093 6455 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm" H 7013 6350 50  0001 C CNN
F 3 "~" H 6975 6500 50  0001 C CNN
	1    6975 6500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 5FAF888D
P 6975 6775
F 0 "#PWR08" H 6975 6525 50  0001 C CNN
F 1 "GND" H 6980 6602 50  0000 C CNN
F 2 "" H 6975 6775 50  0001 C CNN
F 3 "" H 6975 6775 50  0001 C CNN
	1    6975 6775
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR015
U 1 1 5FAF8E39
P 7500 6775
F 0 "#PWR015" H 7500 6525 50  0001 C CNN
F 1 "GND" H 7505 6602 50  0000 C CNN
F 2 "" H 7500 6775 50  0001 C CNN
F 3 "" H 7500 6775 50  0001 C CNN
	1    7500 6775
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR014
U 1 1 5FAF949C
P 7500 5700
F 0 "#PWR014" H 7500 5550 50  0001 C CNN
F 1 "+3.3V" V 7515 5828 50  0000 L CNN
F 2 "" H 7500 5700 50  0001 C CNN
F 3 "" H 7500 5700 50  0001 C CNN
	1    7500 5700
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7500 5700 7500 5825
Wire Wire Line
	7500 6125 7500 6250
Wire Wire Line
	7500 6650 7500 6775
Wire Wire Line
	6975 6650 6975 6775
Wire Wire Line
	6975 6250 7500 6250
Wire Wire Line
	6975 6250 6975 6350
Connection ~ 7500 6250
Wire Wire Line
	7500 6250 7500 6350
$Comp
L Device:R R1
U 1 1 5FB83854
P 6075 6375
F 0 "R1" V 6000 6325 50  0000 L CNN
F 1 "33" V 6075 6325 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6005 6375 50  0001 C CNN
F 3 "~" H 6075 6375 50  0001 C CNN
	1    6075 6375
	0    1    1    0   
$EndComp
Connection ~ 6975 6250
Text GLabel 5775 5100 0    50   Output ~ 0
GPI35
$Comp
L Connector:Screw_Terminal_01x02 J1
U 1 1 5FCBBFB5
P 5175 7025
F 0 "J1" H 5125 7175 50  0000 L CNN
F 1 "AC Current In" H 4950 6775 50  0000 L CNN
F 2 "Connector_Phoenix_MSTB:PhoenixContact_MSTB_2,5_2-GF-5,08_1x02_P5.08mm_Horizontal_ThreadedFlange" H 5175 7025 50  0001 C CNN
F 3 "~" H 5175 7025 50  0001 C CNN
	1    5175 7025
	-1   0    0    -1  
$EndComp
Text Notes 4700 5625 0    59   ~ 12
External current coil
Wire Wire Line
	5375 7025 5725 7025
Wire Wire Line
	5725 7125 5375 7125
Text Notes 5050 7050 2    49   ~ 0
I1
Text Notes 5050 7150 2    49   ~ 0
I2
Wire Wire Line
	5425 6250 5625 6250
$Comp
L ZMCT118F:ZMCT118F L1
U 1 1 5FAE5F7A
P 6075 6700
F 0 "L1" V 6000 6700 50  0000 C CNN
F 1 "TA12" V 6209 6700 50  0000 C CNN
F 2 "Footprints:TA12-100" H 6075 6700 50  0001 C CNN
F 3 "~" H 6075 6700 50  0001 C CNN
	1    6075 6700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6225 6700 6425 6700
Wire Wire Line
	6425 6700 6425 6375
Wire Wire Line
	5425 6350 5425 6700
Wire Wire Line
	5425 6700 5600 6700
Wire Wire Line
	5425 5950 6425 5950
Wire Wire Line
	6225 6375 6425 6375
Connection ~ 6425 6375
Wire Wire Line
	5925 6375 5800 6375
Wire Wire Line
	5800 6375 5800 6700
Connection ~ 5800 6700
Wire Wire Line
	5800 6700 5925 6700
Text GLabel 7975 1450 2    50   Input ~ 0
GPI39
NoConn ~ 7975 1450
Text GLabel 7975 1550 2    50   Input ~ 0
GPI34_BUT1
Text GLabel 8375 3525 2    50   BiDi ~ 0
GPIO15_HS2_CMD
Text GLabel 7975 1750 2    50   BiDi ~ 0
GPIO14_HS2_CLK
NoConn ~ 7975 1750
Text GLabel 4800 1425 0    50   Input ~ 0
ESP_EN
Text GLabel 4800 1525 0    50   BiDi ~ 0
GPIO0
Text GLabel 4800 1625 0    50   BiDi ~ 0
GPIO1_U0TXD
Text GLabel 4800 1725 0    50   BiDi ~ 0
GPIO2_HS2_DATA0
Text GLabel 4800 1825 0    50   BiDi ~ 0
GPIO3_U0RXD
Text GLabel 4800 1925 0    50   BiDi ~ 0
GPIO4_U1TXD
Text GLabel 4800 2025 0    50   BiDi ~ 0
GPIO5_SPI_C5
NoConn ~ 4800 1425
NoConn ~ 4800 1525
NoConn ~ 4800 1625
NoConn ~ 4800 1725
NoConn ~ 4800 1825
NoConn ~ 4800 1925
NoConn ~ 4800 2025
NoConn ~ 7975 1550
NoConn ~ 6875 8675
NoConn ~ 6875 8775
NoConn ~ 6875 8875
NoConn ~ 6875 9375
NoConn ~ 6875 9475
NoConn ~ 6875 9575
NoConn ~ 6875 9675
NoConn ~ 6875 9775
NoConn ~ 5475 8775
NoConn ~ 5475 8875
NoConn ~ 5425 6050
NoConn ~ 5425 6150
Wire Wire Line
	5700 1450 5750 1450
$Comp
L power:+5V #PWR061
U 1 1 5FB5924C
P 15075 900
F 0 "#PWR061" H 15075 750 50  0001 C CNN
F 1 "+5V" H 15090 1073 50  0000 C CNN
F 2 "" H 15075 900 50  0001 C CNN
F 3 "" H 15075 900 50  0001 C CNN
	1    15075 900 
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C8
U 1 1 5FB6CCAE
P 15075 1050
F 0 "C8" H 15193 1096 50  0000 L CNN
F 1 "100uF" H 15193 1005 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm" H 15113 900 50  0001 C CNN
F 3 "~" H 15075 1050 50  0001 C CNN
	1    15075 1050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR062
U 1 1 5FB6CCB4
P 15075 1200
F 0 "#PWR062" H 15075 950 50  0001 C CNN
F 1 "GND" H 15080 1027 50  0000 C CNN
F 2 "" H 15075 1200 50  0001 C CNN
F 3 "" H 15075 1200 50  0001 C CNN
	1    15075 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C9
U 1 1 5FBAB1A3
P 15675 1050
F 0 "C9" H 15790 1096 50  0000 L CNN
F 1 "2n2" H 15790 1005 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D5.1mm_W3.2mm_P5.00mm" H 15713 900 50  0001 C CNN
F 3 "~" H 15675 1050 50  0001 C CNN
	1    15675 1050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR064
U 1 1 5FBAB1AA
P 15675 1200
F 0 "#PWR064" H 15675 950 50  0001 C CNN
F 1 "GND" H 15680 1027 50  0000 C CNN
F 2 "" H 15675 1200 50  0001 C CNN
F 3 "" H 15675 1200 50  0001 C CNN
	1    15675 1200
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR063
U 1 1 5FBD8BA4
P 15675 900
F 0 "#PWR063" H 15675 750 50  0001 C CNN
F 1 "+5V" H 15690 1073 50  0000 C CNN
F 2 "" H 15675 900 50  0001 C CNN
F 3 "" H 15675 900 50  0001 C CNN
	1    15675 900 
	1    0    0    -1  
$EndComp
Connection ~ 6600 5550
$Comp
L power:GND #PWR011
U 1 1 5FBFAE2C
P 7025 5725
F 0 "#PWR011" H 7025 5475 50  0001 C CNN
F 1 "GND" H 7030 5552 50  0000 C CNN
F 2 "" H 7025 5725 50  0001 C CNN
F 3 "" H 7025 5725 50  0001 C CNN
	1    7025 5725
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 5550 7025 5550
Wire Wire Line
	7025 5550 7025 5725
Wire Wire Line
	6600 5450 6600 5550
$Comp
L power:+3.3V #PWR07
U 1 1 5FC2CBDD
P 6600 5150
F 0 "#PWR07" H 6600 5000 50  0001 C CNN
F 1 "+3.3V" H 6475 5300 50  0000 L CNN
F 2 "" H 6600 5150 50  0001 C CNN
F 3 "" H 6600 5150 50  0001 C CNN
	1    6600 5150
	-1   0    0    -1  
$EndComp
$Comp
L Diode:1N4148 D2
U 1 1 5FC31F02
P 6750 5550
F 0 "D2" H 6750 5675 50  0000 C CNN
F 1 "1N4148" H 6800 5450 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 6750 5375 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/1N4148_1N4448.pdf" H 6750 5550 50  0001 C CNN
	1    6750 5550
	1    0    0    -1  
$EndComp
$Comp
L Diode:1N4148 D1
U 1 1 5FC36E44
P 6600 5300
F 0 "D1" V 6500 5050 50  0000 C CNN
F 1 "1N4148" V 6600 5075 50  0000 C CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 6600 5125 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/1N4148_1N4448.pdf" H 6600 5300 50  0001 C CNN
	1    6600 5300
	0    1    1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 5FC7E127
P 6425 5750
F 0 "R3" H 6495 5796 50  0000 L CNN
F 1 "2k2" V 6425 5700 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6355 5750 50  0001 C CNN
F 3 "~" H 6425 5750 50  0001 C CNN
	1    6425 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6425 5900 6425 5950
Connection ~ 6425 5950
Wire Wire Line
	6425 5950 6425 6375
Wire Wire Line
	6425 5600 6425 5550
Wire Wire Line
	6425 5550 6600 5550
Text GLabel 7175 8375 2    50   Output ~ 0
Relay3
Wire Wire Line
	6875 8375 7175 8375
$Comp
L Connector:Screw_Terminal_01x02 J3
U 1 1 5FCD1156
P 5925 7125
F 0 "J3" H 5880 6880 50  0000 L CNN
F 1 "AC Current loop" H 5620 7280 50  0000 L CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal" H 5925 7125 50  0001 C CNN
F 3 "~" H 5925 7125 50  0001 C CNN
	1    5925 7125
	1    0    0    1   
$EndComp
Wire Wire Line
	10275 7250 10475 7250
Text GLabel 10950 7750 2    50   Input ~ 0
230VAC-N
Wire Wire Line
	10950 7750 10875 7750
Wire Wire Line
	10875 7750 10875 7675
Wire Wire Line
	10875 7375 10875 7250
Connection ~ 10875 7250
$Comp
L Jumper:SolderJumper_2_Open JP1
U 1 1 604DFA98
P 10875 7525
F 0 "JP1" V 10829 7593 50  0000 L CNN
F 1 "230VAC-N" V 10920 7593 50  0000 L CNN
F 2 "Footprints:SolderJumper-2_P3mm_Open_RoundedPad" H 10875 7525 50  0001 C CNN
F 3 "~" H 10875 7525 50  0001 C CNN
	1    10875 7525
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J29
U 1 1 5FE6E41C
P 6050 5100
F 0 "J29" H 5975 5325 50  0000 L CNN
F 1 "Current sensor select" H 5650 5425 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6050 5100 50  0001 C CNN
F 3 "~" H 6050 5100 50  0001 C CNN
	1    6050 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5775 5100 5850 5100
Wire Wire Line
	6425 5550 5850 5550
Wire Wire Line
	5850 5550 5850 5200
Connection ~ 6425 5550
$Comp
L Transistor_BJT:2N3906 Q6
U 1 1 60828342
P 8050 2875
F 0 "Q6" V 8378 2875 50  0000 C CNN
F 1 "2N3906" V 8287 2875 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 8250 2800 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/2N3906-D.PDF" H 8050 2875 50  0001 L CNN
	1    8050 2875
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R30
U 1 1 608BD903
P 8050 3300
F 0 "R30" V 7950 3250 50  0000 L CNN
F 1 "1k" V 8050 3250 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 7980 3300 50  0001 C CNN
F 3 "~" H 8050 3300 50  0001 C CNN
	1    8050 3300
	-1   0    0    1   
$EndComp
Wire Wire Line
	8050 3150 8050 3075
Wire Wire Line
	8050 3450 8050 3525
Wire Wire Line
	8050 3525 8375 3525
Wire Wire Line
	8250 2650 8250 2775
Wire Wire Line
	6675 2775 7850 2775
Connection ~ 6675 2775
$Comp
L Connector:TestPoint TP5
U 1 1 60811CBC
P 5600 6650
F 0 "TP5" H 5658 6768 50  0000 L CNN
F 1 "La" H 5658 6677 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_2.0x2.0mm_Drill1.0mm" H 5800 6650 50  0001 C CNN
F 3 "~" H 5800 6650 50  0001 C CNN
	1    5600 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 6650 5600 6700
Connection ~ 5600 6700
Wire Wire Line
	5600 6700 5800 6700
$Comp
L Connector:TestPoint TP6
U 1 1 608599A5
P 6550 6650
F 0 "TP6" H 6608 6768 50  0000 L CNN
F 1 "Lb" H 6608 6677 50  0000 L CNN
F 2 "TestPoint:TestPoint_THTPad_2.0x2.0mm_Drill1.0mm" H 6750 6650 50  0001 C CNN
F 3 "~" H 6750 6650 50  0001 C CNN
	1    6550 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6425 6700 6550 6700
Wire Wire Line
	6550 6700 6550 6650
Connection ~ 6425 6700
Wire Wire Line
	6500 2775 6500 4275
$Comp
L Connector:AudioJack3_SwitchTR J2
U 1 1 5FBF29E7
P 5225 6050
F 0 "J2" H 5207 6375 50  0000 C CNN
F 1 "Current coil in" H 5207 6284 50  0000 C CNN
F 2 "Jack_3.5mm_CUI_FEMPM_Horizontal" H 5225 6050 50  0001 C CNN
F 3 "~" H 5225 6050 50  0001 C CNN
	1    5225 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 7050 9375 7050
Wire Wire Line
	5850 5100 5850 5200
Connection ~ 5850 5100
Connection ~ 5850 5200
Text Notes 4575 4975 0    50   ~ 0
J29 niet monteren, maar \ntussen pin 2 en 3 een \ndoorverbinding aanbrengen
Wire Wire Line
	5425 6350 5625 6350
Wire Wire Line
	5625 6350 5625 6250
Connection ~ 5425 6350
Connection ~ 5625 6250
Wire Wire Line
	5625 6250 6975 6250
Text Notes 3875 6225 0    50   ~ 0
J2 niet monteren, maar\ntussen pin T en TN een\ndoorverbinding aanbrengen
Text GLabel 7975 1850 2    50   BiDi ~ 0
GPIO33
NoConn ~ 7975 1850
Text GLabel 7975 1650 2    50   Input ~ 0
GPI36_U1RXD
NoConn ~ 7975 1650
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 61D4F71F
P 5750 1325
F 0 "#FLG0101" H 5750 1400 50  0001 C CNN
F 1 "PWR_FLAG" H 5625 1500 50  0000 C CNN
F 2 "" H 5750 1325 50  0001 C CNN
F 3 "~" H 5750 1325 50  0001 C CNN
	1    5750 1325
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 1325 5750 1450
Connection ~ 5750 1450
Wire Wire Line
	5750 1450 5850 1450
Text GLabel 11900 3550 2    50   Input ~ 0
230VAC-L
NoConn ~ 11900 3550
$Comp
L power:GND #PWR0101
U 1 1 61D5F4E3
P 13775 1125
F 0 "#PWR0101" H 13775 875 50  0001 C CNN
F 1 "GND" H 13780 952 50  0000 C CNN
F 2 "" H 13775 1125 50  0001 C CNN
F 3 "" H 13775 1125 50  0001 C CNN
	1    13775 1125
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 61D5FC00
P 13775 900
F 0 "#FLG0102" H 13775 975 50  0001 C CNN
F 1 "PWR_FLAG" H 13775 1075 50  0000 C CNN
F 2 "" H 13775 900 50  0001 C CNN
F 3 "~" H 13775 900 50  0001 C CNN
	1    13775 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	13775 900  13775 1125
NoConn ~ 5850 5000
Text GLabel 10950 7975 2    50   Input ~ 0
230VAC-N
NoConn ~ 10950 7975
Text GLabel 7825 8275 2    50   Output ~ 0
Relay2
Text GLabel 8925 4375 0    50   Input ~ 0
FET1
Text GLabel 7825 8575 2    50   Output ~ 0
FET2
Text GLabel 7825 8375 2    50   Output ~ 0
Relay3
NoConn ~ 7825 8275
NoConn ~ 7825 8375
NoConn ~ 7825 8575
Text GLabel 7800 9075 2    50   Input ~ 0
Switch1
Text GLabel 7800 9175 2    50   Input ~ 0
Switch2
Text GLabel 7800 9275 2    50   Input ~ 0
Switch3
NoConn ~ 7800 9075
NoConn ~ 7800 9175
NoConn ~ 7800 9275
$Comp
L Device:R R22
U 1 1 61B66960
P 9100 4625
F 0 "R22" H 9170 4671 50  0000 L CNN
F 1 "1k5" V 9100 4575 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 9030 4625 50  0001 C CNN
F 3 "~" H 9100 4625 50  0001 C CNN
	1    9100 4625
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D13
U 1 1 61B68196
P 9100 5100
F 0 "D13" V 9139 4983 50  0000 R CNN
F 1 "LED" V 9048 4983 50  0000 R CNN
F 2 "LED_THT:LED_D3.0mm" H 9100 5100 50  0001 C CNN
F 3 "~" H 9100 5100 50  0001 C CNN
	1    9100 5100
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 61B68F30
P 9100 5375
F 0 "#PWR0102" H 9100 5125 50  0001 C CNN
F 1 "GND" H 9105 5202 50  0000 C CNN
F 2 "" H 9100 5375 50  0001 C CNN
F 3 "" H 9100 5375 50  0001 C CNN
	1    9100 5375
	1    0    0    -1  
$EndComp
Wire Wire Line
	8925 4375 9100 4375
Wire Wire Line
	9100 4375 9100 4475
Wire Wire Line
	9100 4775 9100 4950
Wire Wire Line
	9100 5250 9100 5375
$EndSCHEMATC
