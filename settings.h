#ifndef SETTINGS_H
#define SETTINGS_H


/*
	All the permanent settings stored in the flash are here.

	Calibration values, control loop constants, maybe some statistics...

	This is all stored in flash but copied to ram in boot code, you can use everything normally:
	just call save_settings() to write them back to flash.

	Just keep in mind that the flash has limited number of erase/write cycles.
	You can, for example, rewrite the flash once per hour, and/or when something critical has changed.

	save_settings() is blocking and slow.

	Currently, a 16K flash page is reserved for the settings.
*/

typedef struct
{
	const uint32_t magic;	// for easily identifying a settings page
	const uint32_t version; // In the future, we might want to provide advanced firmware update which checks
	                        // for compatibility of the settings section to bring over old settings if possible.
	                        // Change the version number when the settings are incompatible with older revisions,
	                        // e.g., if variable sizes or offsets change


	
} settings_t;

extern settings_t settings __attribute__((section(".settings")));

void save_settings();


#endif
