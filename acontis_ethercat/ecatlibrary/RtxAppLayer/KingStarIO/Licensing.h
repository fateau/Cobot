// Licensing.h
//
// Copyright (c) 2012-2018 IntervalZero Inc., All Rights Reserved
//
// Author: Alex Filippov
//
// This file contains type definitions that are to be used for licensing related public and internal API's
//
// IMPORTANT: Only use C syntax in this file, because it is likely to be included into .c files.

#ifndef __RTX_LICENSING__
#define __RTX_LICENSING__ 1

#include "ErrorCodes.h"

#ifdef __cplusplus
extern "C"
{
#endif

// Maximum size of the options string in the license data
#define RT_MAX_LICENSE_OPTIONS_LENGTH 64

// Maximum length of a feature name 
#define RT_MAX_FEATURE_NAME_LENGTH 64

// Maximum length of the dongle ID
#define DONGLE_ID_MAX_LENGTH 66

// Version information.
typedef struct _RT_VERSION_INFO
{
	int majorVersion;
	int minorVersion;
	int fixVersion;
	int buildNumber;
} RT_VERSION_INFO, *PRT_VERSION_INFO;

// Possible RtGetLicenses API license statuses
typedef enum _RT_FEATURE_LICENSE_STATUS      
{
	// License entry not found
	RT_FEATURE_STATUS_NOT_FOUND = 0,

	// License entry found and it's in invalid format
	RT_FEATURE_STATUS_FORMAT_ERROR,

	// License entry found and it's valid
	RT_FEATURE_STATUS_VALID,

	// License entry found and it's invalid
	RT_FEATURE_STATUS_INVALID,

	// License entry found and it's evaluation. 
	RT_FEATURE_STATUS_EVAL,

	// License entry found and it's expired evaluation. 
	RT_FEATURE_STATUS_EVAL_EXPIRED,

	// License entry found and this is an evaluation dongle based license for which host id is not present on the system
	RT_FEATURE_STATUS_EVAL_INVALID_HOST_ID,

	// License entry found and this is an permanent dongle based license for which host id is not present on the system
	RT_FEATURE_STATUS_INVALID_HOST_ID
} RT_FEATURE_LICENSE_STATUS, * PRT_FEATURE_LICENSE_STATUS;

// This structure represents license expiration date
typedef struct _RT_LICENSE_EXPIRATION_DATE
{
	unsigned int expirationDateYear;	
	unsigned int expirationDateMonth;	
	unsigned int expirationDateDay;	
	unsigned int daysUntilExpiration;
} RT_LICENSE_EXPIRATION_DATE, * PRT_LICENSE_EXPIRATION_DATE;

// This structure represents license issue date
typedef struct _RT_LICENSE_ISSUE_DATE
{
	unsigned int issueDateYear;	
	unsigned int issueDateMonth;	
	unsigned int issueDateDay;	
} RT_LICENSE_ISSUE_DATE, * PRT_LICENSE_ISSUE_DATE;

// License options ASCII
typedef struct _RT_LICENSE_OPTIONS_A
{
	char Raw[RT_MAX_LICENSE_OPTIONS_LENGTH];    // Raw RLM product options string.
	int RtProcessors;							// Number of real time processors.
	int RtCustomerId;							// Customer ID for this license.
} RT_LICENSE_OPTIONS_A, * PRT_LICENSE_OPTIONS_A;
 
// License option UNICODE
typedef struct _RT_LICENSE_OPTIONS_W
{
	wchar_t Raw[RT_MAX_LICENSE_OPTIONS_LENGTH];    // Raw RLM product options string.
	int RtProcessors;							 // Number of real time processors.
	int RtCustomerId;							 // Customer ID for this license.	
} RT_LICENSE_OPTIONS_W, * PRT_LICENSE_OPTIONS_W;
 
// Data returned by call to retrieve license info ASCII.
typedef struct _RT_LICENSE_INFO_A 
{
	int size;										// Size of the structure
	int structureVersion;							// Version of this structure.
	char featureName [RT_MAX_FEATURE_NAME_LENGTH];	// Name the licensed feature.
	unsigned int majorVersion;						// Major version of the licensed feature.
	RT_FEATURE_LICENSE_STATUS status;				// License status.
	int isFeatureInstalled;							// Specifies if feature is installed
	RT_LICENSE_EXPIRATION_DATE expirationDate;			// Represents expiration date 
	RT_LICENSE_ISSUE_DATE issueDate;					// Issue Date
	char dongleSerialNumber[DONGLE_ID_MAX_LENGTH];	// Dongle name the license is nodelocked against.
	RT_LICENSE_OPTIONS_A options;						// RLM Options. 
} RT_LICENSE_INFO_A, * PRT_LICENSE_INFO_A;

// Data returned by call to retrieve license info UNICODE.
typedef struct _RT_LICENSE_INFO_W 
{
	int size;										// Size of the structure
	int structureVersion;							// Version of this structure.
	wchar_t featureName [RT_MAX_FEATURE_NAME_LENGTH];	// Name the licensed feature.
	unsigned int majorVersion;						// Major version of the licensed feature.
	RT_FEATURE_LICENSE_STATUS status;				// License status.
	int isFeatureInstalled;							// Specifies if feature is installed
	RT_LICENSE_EXPIRATION_DATE expirationDate;			// Represents expiration date 
	RT_LICENSE_ISSUE_DATE issueDate;					// Issue Date
	wchar_t dongleSerialNumber[DONGLE_ID_MAX_LENGTH]; // Dongle name the license is nodelocked against.
	RT_LICENSE_OPTIONS_W options; 						// RLM Options. 
} RT_LICENSE_INFO_W, * PRT_LICENSE_INFO_W;

#ifdef UNICODE
#define RT_LICENSE_INFO RT_LICENSE_INFO_W
#define PRT_LICENSE_INFO PRT_LICENSE_INFO_W
#define RT_LICENSE_OPTIONS RT_LICENSE_OPTIONS_W
#else
#define RT_LICENSE_INFO RT_LICENSE_INFO_A
#define PRT_LICENSE_INFO PRT_LICENSE_INFO_A
#define RT_LICENSE_OPTIONS RT_LICENSE_OPTIONS_A
#endif

// Pre-defined feature names.  These can be passed to RtGetLicenseFeatureStatus() and RtGetLicenseFeatureStatusEx().

#ifdef UNICODE
# define RTXRUNTIME_FEATURE_NAME        L"IZRTX64"    // The runtime feature.
# define RTXTCPIP_FEATURE_NAME          L"IZTCP64"    // The TCP/IP stack feature.
#else
# define RTXRUNTIME_FEATURE_NAME        "IZRTX64"     // The runtime feature.
# define RTXTCPIP_FEATURE_NAME          "IZTCP64"     // The TCP/IP stack feature.
#endif

#ifdef __cplusplus 
}
#endif

#endif
