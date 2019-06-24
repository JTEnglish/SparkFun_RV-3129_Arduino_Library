/******************************************************************************
SparkFun_RV1805.h
RV1805 Arduino Library
Andy England @ SparkFun Electronics
February 5, 2018
https://github.com/sparkfun/Qwiic_RTC

Development environment specifics:
Arduino IDE 1.6.4

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/

#include "SparkFun_RV3129.h"

//****************************************************************************//
//
//  Settings and configuration
//
//****************************************************************************//

// Parse the __DATE__ predefined macro to generate date defaults:
// __Date__ Format: MMM DD YYYY (First D may be a space if <10)
// <MONTH>
#define BUILD_MONTH_JAN ((__DATE__[0] == 'J') && (__DATE__[1] == 'a')) ? 1 : 0
#define BUILD_MONTH_FEB (__DATE__[0] == 'F') ? 2 : 0
#define BUILD_MONTH_MAR ((__DATE__[0] == 'M') && (__DATE__[1] == 'a') && (__DATE__[2] == 'r')) ? 3 : 0
#define BUILD_MONTH_APR ((__DATE__[0] == 'A') && (__DATE__[1] == 'p')) ? 4 : 0
#define BUILD_MONTH_MAY ((__DATE__[0] == 'M') && (__DATE__[1] == 'a') && (__DATE__[2] == 'y')) ? 5 : 0
#define BUILD_MONTH_JUN ((__DATE__[0] == 'J') && (__DATE__[1] == 'u') && (__DATE__[2] == 'n')) ? 6 : 0
#define BUILD_MONTH_JUL ((__DATE__[0] == 'J') && (__DATE__[1] == 'u') && (__DATE__[2] == 'l')) ? 7 : 0
#define BUILD_MONTH_AUG ((__DATE__[0] == 'A') && (__DATE__[1] == 'u')) ? 8 : 0
#define BUILD_MONTH_SEP (__DATE__[0] == 'S') ? 9 : 0
#define BUILD_MONTH_OCT (__DATE__[0] == 'O') ? 10 : 0
#define BUILD_MONTH_NOV (__DATE__[0] == 'N') ? 11 : 0
#define BUILD_MONTH_DEC (__DATE__[0] == 'D') ? 12 : 0
#define BUILD_MONTH BUILD_MONTH_JAN | BUILD_MONTH_FEB | BUILD_MONTH_MAR | \
BUILD_MONTH_APR | BUILD_MONTH_MAY | BUILD_MONTH_JUN | \
BUILD_MONTH_JUL | BUILD_MONTH_AUG | BUILD_MONTH_SEP | \
BUILD_MONTH_OCT | BUILD_MONTH_NOV | BUILD_MONTH_DEC
// <DATE>
#define BUILD_DATE_0 ((__DATE__[4] == ' ') ? 0 : (__DATE__[4] - 0x30))
#define BUILD_DATE_1 (__DATE__[5] - 0x30)
#define BUILD_DATE ((BUILD_DATE_0 * 10) + BUILD_DATE_1)
// <YEAR>
#define BUILD_YEAR (((__DATE__[7] - 0x30) * 1000) + ((__DATE__[8] - 0x30) * 100) + \
((__DATE__[9] - 0x30) * 10)  + ((__DATE__[10] - 0x30) * 1))

// Parse the __TIME__ predefined macro to generate time defaults:
// __TIME__ Format: HH:MM:SS (First number of each is padded by 0 if <10)
// <HOUR>
#define BUILD_HOUR_0 ((__TIME__[0] == ' ') ? 0 : (__TIME__[0] - 0x30))
#define BUILD_HOUR_1 (__TIME__[1] - 0x30)
#define BUILD_HOUR ((BUILD_HOUR_0 * 10) + BUILD_HOUR_1)
// <MINUTE>
#define BUILD_MINUTE_0 ((__TIME__[3] == ' ') ? 0 : (__TIME__[3] - 0x30))
#define BUILD_MINUTE_1 (__TIME__[4] - 0x30)
#define BUILD_MINUTE ((BUILD_MINUTE_0 * 10) + BUILD_MINUTE_1)
// <SECOND>
#define BUILD_SECOND_0 ((__TIME__[6] == ' ') ? 0 : (__TIME__[6] - 0x30))
#define BUILD_SECOND_1 (__TIME__[7] - 0x30)
#define BUILD_SECOND ((BUILD_SECOND_0 * 10) + BUILD_SECOND_1)

RV3129::RV3129( void )
{

}

boolean RV3129::begin(TwoWire &wirePort)
{
	//We require caller to begin their I2C port, with the speed of their choice
	//external to the library
	//_i2cPort->begin();
	_i2cPort = &wirePort;

	// enableTrickleCharge();
	// enableLowPower();

	// uint8_t setting = readRegister(RV3129_CTRL1);
	// setting |= CTRL1_ARST; //Enables clearing of interrupt flags upon read of status register
	// writeRegister(RV3129_CTRL1, setting);

	set12Hour();

	return(true);
}

//Configure RTC to output 1-12 hours
//Converts any current hour setting to 12 hour
void RV3129::set12Hour()
{
	//Do we need to change anything?
	if(is12Hour() == false)
	{		
		uint8_t hour = BCDtoDEC(readRegister(RV3129_HOURS)); //Get the current hour in the RTC

		//Take the current hours and convert to 12, complete with AM/PM bit
		boolean pm = false;

		if(hour == 0)
			hour += 12;
		else if(hour == 12)
			pm = true;
		else if(hour > 12)
		{
			hour -= 12;
			pm = true;
		}
		
		hour = DECtoBCD(hour); //Convert to BCD
		//Set the 12/24 hour bit to 1
		hour |= (1<<HOURS_12_24);

		if(pm == true) hour |= (1<<HOURS_AM_PM); //Set AM/PM bit if needed

		writeRegister(RV3129_HOURS, hour); //Record this to hours register
	}
}

//Configure RTC to output 0-23 hours
//Converts any current hour setting to 24 hour
void RV3129::set24Hour()
{
	//Do we need to change anything?
	if(is12Hour() == true)
	{		
		//Not sure what changing the CTRL1 register will do to hour register so let's get a copy
		uint8_t hour = readRegister(RV3129_HOURS); //Get the current 12 hour formatted time in BCD
		boolean pm = false;
		if(hour & (1<<HOURS_AM_PM)) //Is the AM/PM bit set?
		{
			pm = true;
			hour &= ~(1<<HOURS_AM_PM); //Clear the bit
		}

		//Given a BCD hour in the 1-12 range, make it 24
		hour = BCDtoDEC(hour); //Convert core of register to DEC
		
		if(pm == true) hour += 12; //2PM becomes 14
		if(hour == 12) hour = 0; //12AM stays 12, but should really be 0
		if(hour == 24) hour = 12; //12PM becomes 24, but should really be 12

		hour = DECtoBCD(hour); //Convert to BCD
		//Change to 24 hour mode
		hour &= ~(1<<HOURS_12_24);

		writeRegister(RV3129_HOURS, hour); //Record this to hours register
	}
}

//Returns true if RTC has been configured for 12 hour mode
bool RV3129::is12Hour()
{
	uint8_t hoursVal = readRegister(RV3129_HOURS);
	return(hoursVal & (1 << HOURS_12_24));
}

//Returns true if RTC has PM bit set and 12Hour bit set
bool RV3129::isPM()
{
	uint8_t hourRegister = readRegister(RV3129_HOURS);
	if(is12Hour() && (hourRegister & (1<<HOURS_AM_PM)))
		return(true);
	return(false);
}

//Returns a pointer to array of chars that are the date in mm/dd/yyyy format because we're weird
char* RV3129::stringDateUSA()
{
	static char date[11]; //Max of mm/dd/yyyy with \0 terminator
	sprintf(date, "%02d/%02d/20%02d", BCDtoDEC(_time[TIME_MONTH]), BCDtoDEC(_time[TIME_DATE]), BCDtoDEC(_time[TIME_YEAR]));
	return(date);
}

//Returns a pointer to array of chars that are the date in dd/mm/yyyy format
char*  RV3129::stringDate()
{
	static char date[11]; //Max of dd/mm/yyyy with \0 terminator
	sprintf(date, "%02d/%02d/20%02d", BCDtoDEC(_time[TIME_DATE]), BCDtoDEC(_time[TIME_MONTH]), BCDtoDEC(_time[TIME_YEAR]));
	return(date);
}

//Returns a pointer to array of chars that represents the time in hh:mm:ss format
//Adds AM/PM if in 12 hour mode
char* RV3129::stringTime()
{
	static char time[11]; //Max of hh:mm:ssXM with \0 terminator

	if(is12Hour() == true)
	{
		char half = 'A';
		if(isPM()) half = 'P';
		
		sprintf(time, "%02d:%02d:%02d%cM", BCDtoDEC(_time[TIME_HOURS]), BCDtoDEC(_time[TIME_MINUTES]), BCDtoDEC(_time[TIME_SECONDS]), half);
	}
	else sprintf(time, "%02d:%02d:%02d", BCDtoDEC(_time[TIME_HOURS]), BCDtoDEC(_time[TIME_MINUTES]), BCDtoDEC(_time[TIME_SECONDS]));
	
	return(time);
}

char* RV3129::stringTimeStamp()
{
	static char timeStamp[23]; //Max of yyyy-mm-ddThh:mm:ss.ss with \0 terminator

	sprintf(timeStamp, "20%02d-%02d-%02dT%02d:%02d:%02d", BCDtoDEC(_time[TIME_YEAR]), BCDtoDEC(_time[TIME_MONTH]), BCDtoDEC(_time[TIME_DATE]), BCDtoDEC(_time[TIME_HOURS]), BCDtoDEC(_time[TIME_MINUTES]), BCDtoDEC(_time[TIME_SECONDS]));
	
	return(timeStamp);
}

char* RV3129::stringTemp() {
	static char tempString[3];
	uint8_t tempVal = getTemp();

	if (tempVal < 60) {
		sprintf(tempString, "-%d", (60 - tempVal));
	}
	else {
		sprintf(tempString, "%d", (tempVal - 60));
	}

	return tempString;
}

bool RV3129::setTime(uint8_t sec, uint8_t min, uint8_t hour, uint8_t date, uint8_t month, uint16_t year, uint8_t day)
{
	_time[TIME_SECONDS] = DECtoBCD(sec);
	_time[TIME_MINUTES] = DECtoBCD(min);
	_time[TIME_HOURS] = DECtoBCD(hour);
	_time[TIME_DATE] = DECtoBCD(date);
	_time[TIME_MONTH] = DECtoBCD(month);
	_time[TIME_YEAR] = DECtoBCD(year - 2000);
	_time[TIME_DAY] = DECtoBCD(day);
	
	bool status = false;
	
	if (is12Hour())
	{
		set24Hour();
		status = setTime(_time, TIME_ARRAY_LENGTH);
		set12Hour();
	}
	else
	{
		status = setTime(_time, TIME_ARRAY_LENGTH);
	}
	return status;
}

// setTime -- Set time and date/day registers of RV3129 (using data array)
bool RV3129::setTime(uint8_t * time, uint8_t len)
{
	if (len != TIME_ARRAY_LENGTH)
		return false;
	
	return writeMultipleRegisters(RV3129_SECONDS, time, len);
}

bool RV3129::setSeconds(uint8_t value)
{
	_time[TIME_SECONDS] = DECtoBCD(value);
	return setTime(_time, TIME_ARRAY_LENGTH);
}

bool RV3129::setMinutes(uint8_t value)
{
	_time[TIME_MINUTES] = DECtoBCD(value);
	return setTime(_time, TIME_ARRAY_LENGTH);
}

bool RV3129::setHours(uint8_t value)
{
	_time[TIME_HOURS] = DECtoBCD(value);
	return setTime(_time, TIME_ARRAY_LENGTH);
}

bool RV3129::setDate(uint8_t value)
{
	_time[TIME_DATE] = DECtoBCD(value);
	return setTime(_time, TIME_ARRAY_LENGTH);
}

bool RV3129::setMonth(uint8_t value)
{
	_time[TIME_MONTH] = DECtoBCD(value);
	return setTime(_time, TIME_ARRAY_LENGTH);
}

bool RV3129::setYear(uint8_t value)
{
	_time[TIME_YEAR] = DECtoBCD(value);
	return setTime(_time, TIME_ARRAY_LENGTH);
}

bool RV3129::setWeekday(uint8_t value)
{
	_time[TIME_DAY] = DECtoBCD(value);
	return setTime(_time, TIME_ARRAY_LENGTH);
}

//Move the hours, mins, sec, etc registers from RV-1805 into the _time array
//Needs to be called before printing time or date
//We do not protect the GPx registers. They will be overwritten. The user has plenty of RAM if they need it.
bool RV3129::updateTime()
{
	if (readMultipleRegisters(RV3129_SECONDS, _time, TIME_ARRAY_LENGTH) == false)
		return(false); //Something went wrong
	
	if(is12Hour()) _time[TIME_HOURS] &= ~(1<<HOURS_AM_PM); //Remove this bit from value
	
	return true;
}

uint8_t RV3129::getSeconds()
{
	return BCDtoDEC(_time[TIME_SECONDS]);
}

uint8_t RV3129::getMinutes()
{
	return BCDtoDEC(_time[TIME_MINUTES]);
}

uint8_t RV3129::getHours()
{
	return BCDtoDEC(_time[TIME_HOURS]);
}

uint8_t RV3129::getWeekday()
{
	return BCDtoDEC(_time[TIME_DAY]);
}

uint8_t RV3129::getDate()
{
	return BCDtoDEC(_time[TIME_DATE]);
}

uint8_t RV3129::getMonth()
{
	return BCDtoDEC(_time[TIME_MONTH]);
}

uint8_t RV3129::getYear()
{
	return BCDtoDEC(_time[TIME_YEAR]);
}

uint8_t RV3129::getTemp()
{
	// Value Range: -60 to +194 Deg. C
	// Value Map:
	// Deg. C 			->	-60		0		190
	// Int Val (Dec.)	->	0		60		250
	return readRegister(RV3129_TEMP);
}

bool RV3129::setAlarm(uint8_t sec, uint8_t min, uint8_t hour, uint8_t date, uint8_t week_day, uint8_t month, uint8_t year)
{
	uint8_t alarmTime[ALARM_ARRAY_LENGTH];
	
	alarmTime[TIME_SECONDS] = DECtoBCD(sec);
	alarmTime[TIME_MINUTES] = DECtoBCD(min);
	alarmTime[TIME_HOURS] = DECtoBCD(hour);
	alarmTime[TIME_DATE] = DECtoBCD(date);
	alarmTime[TIME_DAY] = DECtoBCD(week_day);
	alarmTime[TIME_MONTH] = DECtoBCD(month);
	alarmTime[TIME_YEAR] = DECtoBCD(year);
	
	return setAlarm(alarmTime, ALARM_ARRAY_LENGTH);
}

bool RV3129::setAlarm(uint8_t * alarmTime, uint8_t len)
{
	if (len != ALARM_ARRAY_LENGTH)
		return false;
	
	return writeMultipleRegisters(RV3129_SECONDS_ALM, alarmTime, ALARM_ARRAY_LENGTH);
}

bool RV3129::enableDisableAlarm(uint8_t enableBits) {
	uint8_t alarmTime[ALARM_ARRAY_LENGTH];
	if ( !readMultipleRegisters(uint8_t addr, uint8_t * dest, uint8_t len) ) {
		return false;
	}

	alarmTime[TIME_SECONDS] |= (((enableBits >> TIME_SECONDS) & 1 ) << 7);
	alarmTime[TIME_MINUTES] |= (((enableBits >> TIME_MINUTES) & 1 ) << 7);
	alarmTime[TIME_HOURS]   |= (((enableBits >> TIME_HOURS) & 1 ) << 7);
	alarmTime[TIME_DATE] 	|= (((enableBits >> TIME_DATE) & 1 ) << 7);
	alarmTime[TIME_DAY] 	|= (((enableBits >> TIME_DAY) & 1 ) << 7);
	alarmTime[TIME_MONTH] 	|= (((enableBits >> TIME_MONTH) & 1 ) << 7);
	alarmTime[TIME_YEAR] 	|= (((enableBits >> TIME_YEAR) & 1 ) << 7);

	return writeMultipleRegisters(RV3129_SECONDS_ALM, alarmTime, ALARM_ARRAY_LENGTH);
}

//Takes the time from the last build and uses it as the current time
//Works very well as an arduino sketch
bool RV3129::setToCompilerTime()
{
	_time[TIME_SECONDS] = DECtoBCD(BUILD_SECOND);
	_time[TIME_MINUTES] = DECtoBCD(BUILD_MINUTE);
	_time[TIME_HOURS] = DECtoBCD(BUILD_HOUR);

	//Build_Hour is 0-23, convert to 1-12 if needed
	if (is12Hour())
	{
		uint8_t hour = BUILD_HOUR;

		boolean pm = false;

		if(hour == 0)
			hour += 12;
		else if(hour == 12)
			pm = true;
		else if(hour > 12)
		{
			hour -= 12;
			pm = true;
		}

		_time[TIME_HOURS] = DECtoBCD(hour); //Load the modified hours
	
		if(pm == true) _time[TIME_HOURS] |= (1<<HOURS_AM_PM); //Set AM/PM bit if needed
	}
	
	_time[TIME_MONTH] = DECtoBCD(BUILD_MONTH);
	_time[TIME_DATE] = DECtoBCD(BUILD_DATE);
	_time[TIME_YEAR] = DECtoBCD(BUILD_YEAR - 2000); //! Not Y2K (or Y2.1K)-proof :(
	
	// Calculate weekday (from here: http://stackoverflow.com/a/21235587)
	// 0 = Sunday, 6 = Saturday
	uint16_t d = BUILD_DATE;
	uint16_t m = BUILD_MONTH;
	uint16_t y = BUILD_YEAR;
	uint16_t weekday = (d+=m<3?y--:y-2,23*m/9+d+4+y/4-y/100+y/400)%7 + 1;
	_time[TIME_DAY] = DECtoBCD(weekday);
	
	return setTime(_time, TIME_ARRAY_LENGTH);
}

uint8_t RV3129::BCDtoDEC(uint8_t val)
{
	return ( ( val / 0x10) * 10 ) + ( val % 0x10 );
}

// BCDtoDEC -- convert decimal to binary-coded decimal (BCD)
uint8_t RV3129::DECtoBCD(uint8_t val)
{
	return ( ( val / 10 ) * 0x10 ) + ( val % 10 );
}

uint8_t RV3129::readRegister(uint8_t addr)
{
	_i2cPort->beginTransmission(RV3129_ADDR);
	_i2cPort->write(addr);
	_i2cPort->endTransmission();

	_i2cPort->requestFrom(RV3129_ADDR, (uint8_t)1);
	if (_i2cPort->available()) {
		return (_i2cPort->read());
	}
	else {
		return (0xFF); //Error
	}
}

bool RV3129::writeRegister(uint8_t addr, uint8_t val)
{
	_i2cPort->beginTransmission(RV3129_ADDR);
	_i2cPort->write(addr);
	_i2cPort->write(val);
    if (_i2cPort->endTransmission() != 0)
      return (false); //Error: Sensor did not ack
	return(true);
}

bool RV3129::writeMultipleRegisters(uint8_t addr, uint8_t * values, uint8_t len)
{
	_i2cPort->beginTransmission(RV3129_ADDR);
	_i2cPort->write(addr);
	for (uint8_t i = 0; i < len; i++)
	{
		_i2cPort->write(values[i]);
	}

    if (_i2cPort->endTransmission() != 0)
      return (false); //Error: Sensor did not ack
	return(true);
}

bool RV3129::readMultipleRegisters(uint8_t addr, uint8_t * dest, uint8_t len)
{
	_i2cPort->beginTransmission(RV3129_ADDR);
	_i2cPort->write(addr);
    if (_i2cPort->endTransmission() != 0)
      return (false); //Error: Sensor did not ack

	_i2cPort->requestFrom(RV3129_ADDR, len);
	for (uint8_t i = 0; i < len; i++)
	{
		dest[i] = _i2cPort->read();
	}
	
	return(true);
}