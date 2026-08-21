/*
 * ClosedLoop.cpp
 *
 *  Created on: 19 Mar 2021
 *      Author: Louis
 */

#include "ClosedLoop.h"

#if SUPPORT_CAN_EXPANSION && (HAS_MASS_STORAGE || HAS_SBC_INTERFACE)

# include <Platform/RepRap.h>
# include <CAN/CanInterface.h>
# include <Platform/Platform.h>
# include <CanMessageFormats.h>
# include <Storage/MassStorage.h>
# include <CAN/ExpansionManager.h>
# include <GCodes/GCodeBuffer/GCodeBuffer.h>
# include <CAN/CanMessageGenericConstructor.h>
# include <General/Portability.h>
# include <atomic>
# include <limits>

constexpr unsigned int MaxSamples = 65535;				// This comes from the fact CanMessageClosedLoopData->firstSampleNumber has a max value of 65535
constexpr uint32_t DataReceiveTimeout = 5000;			// Data receive timeout in milliseconds

static uint16_t rateRequested;							// The sampling rate
static uint8_t modeRequested;							// The sampling mode(immediate or on next move)
static uint32_t filterRequested;						// A filter for what data is collected
static size_t dataBytesPerSample;						// How many bytes there will be in each sample
static DriverId deviceRequested;						// The driver being sampled
static uint8_t movementRequested;						// The movement to be made whilst recording
static std::atomic<uint32_t> whenDataLastReceived;
static std::atomic<uint32_t> numSamplesRequested;					// The number of samples to collect
static std::atomic<FileStore *_ecv_null> closedLoopFile(nullptr);	// This is non-null when the data collection is running, null otherwise

static unsigned int expectedRemoteSampleNumber = 0;
static CanAddress expectedRemoteBoardAddress = CanId::NoAddress;

// Description of every recordable closed-loop variable, in CL_RECORD_ bit order.
//
// This is the single source of truth for both the CSV heading line and the decoding of received samples.
// Those used to be two separate if-chains that had to be kept in the same order by hand, and an earlier
// attempt to extend them failed partly because a heading array was written in a different order from the
// bits and then "corrected" with a bitmask-shuffling expression. Driving both from one table in bit
// order removes the possibility of that class of mistake: a new channel is one row here, plus the
// matching size entry in Duet3Common.h and the matching write in the expansion board's CollectSample().
//
// The order MUST match the CL_RECORD_ bit order and ClosedLoopDataSizes[] in Duet3Common.h - that is the
// order the expansion board writes fields to the wire.
enum class ClFieldKind : uint8_t { i32, f32, f16, u16, i16 };

struct ClosedLoopChannel
{
	uint32_t bit;
	const char *_ecv_array heading;
	ClFieldKind kind;
	const char *_ecv_array format;			// printf format for the value, excluding the leading comma
};

static constexpr ClosedLoopChannel ClosedLoopChannels[] =
{
	{ CL_RECORD_RAW_ENCODER_READING,	"Raw Encoder Reading",	ClFieldKind::i32,	"%" PRIi32 },
	{ CL_RECORD_CURRENT_MOTOR_STEPS,	"Measured Motor Steps",	ClFieldKind::f32,	"%.2f" },
	{ CL_RECORD_TARGET_MOTOR_STEPS,		"Target Motor Steps",	ClFieldKind::f32,	"%.2f" },
	{ CL_RECORD_CURRENT_ERROR,			"Current Error",		ClFieldKind::f32,	"%.2f" },
	{ CL_RECORD_PID_CONTROL_SIGNAL,		"PID Control Signal",	ClFieldKind::f16,	"%.1f" },
	// 4 decimal places rather than 1: this matches the real precision available in the underlying
	// half-float storage, and costs nothing on the PID terms' larger dynamic range.
	{ CL_RECORD_PID_P_TERM,				"PID P Term",			ClFieldKind::f16,	"%.4f" },
	{ CL_RECORD_PID_I_TERM,				"PID I Term",			ClFieldKind::f16,	"%.4f" },
	{ CL_RECORD_PID_D_TERM,				"PID D Term",			ClFieldKind::f16,	"%.4f" },
	{ CL_RECORD_CURRENT_STEP_PHASE,		"Measured Step Phase",	ClFieldKind::u16,	"%u" },
	{ CL_RECORD_DESIRED_STEP_PHASE,		"Desired Step Phase",	ClFieldKind::u16,	"%u" },
	// u16 like the two step-phase channels above it: the expansion board writes PutU16, and phase shift is
	// an angle in the same units. Currently always 0 - the channel has never had a source.
	{ CL_RECORD_PHASE_SHIFT,			"Phase Shift",			ClFieldKind::u16,	"%u" },
	{ CL_RECORD_COIL_A_CURRENT,			"Coil A Current",		ClFieldKind::i16,	"%d" },
	{ CL_RECORD_COIL_B_CURRENT,			"Coil B Current",		ClFieldKind::i16,	"%d" },
	{ CL_RECORD_PID_V_TERM,				"PID V Term",			ClFieldKind::f16,	"%.1f" },
	{ CL_RECORD_PID_A_TERM,				"PID A Term",			ClFieldKind::f16,	"%.1f" },
	{ CL_RECORD_PID_J_TERM,				"PID J Term",			ClFieldKind::f16,	"%.1f" },
	{ CL_RECORD_MEASURED_VELOCITY,		"Measured Velocity",	ClFieldKind::f16,	"%.4f" },
	{ CL_RECORD_PHASE_CURRENT_A,		"Phase Current A",		ClFieldKind::f16,	"%.4f" },
	{ CL_RECORD_PHASE_CURRENT_B,		"Phase Current B",		ClFieldKind::f16,	"%.4f" },
	{ CL_RECORD_PHASE_CURRENT_C,		"Phase Current C",		ClFieldKind::f16,	"%.4f" },
	{ CL_RECORD_CURRENT_D,				"Current D",			ClFieldKind::f16,	"%.4f" },
	{ CL_RECORD_CURRENT_Q,				"Current Q",			ClFieldKind::f16,	"%.4f" },
	{ CL_RECORD_VOLTAGE_D,				"Voltage D",			ClFieldKind::f16,	"%.4f" },
	{ CL_RECORD_VOLTAGE_Q,				"Voltage Q",			ClFieldKind::f16,	"%.4f" }
};

static_assert(ARRAY_SIZE(ClosedLoopChannels) == NumClosedLoopRecordChannels,
				"ClosedLoopChannels must describe every CL_RECORD_ channel");

static bool OpenDataCollectionFile(const char *_ecv_array filename, unsigned int size) noexcept
{
	// Create the file
	FileStore *_ecv_null const f = MassStorage::OpenFile(filename, OpenMode::write, size);
	if (f == nullptr) { return false; }

	// Write the header line. Driven from ClosedLoopChannels[] so that the column order here cannot drift
	// away from the order ProcessReceivedData() decodes them in.
	{
		String<StringLength500> temp;
		temp.copy("Sample,Timestamp");
		for (const ClosedLoopChannel& chan : ClosedLoopChannels)
		{
			if (filterRequested & chan.bit)
			{
				temp.cat(',');
				temp.cat(chan.heading);
			}
		}

		temp.cat("\n");
		f->Write(temp.c_str());							// this call could result in the file becoming invalidated
	}

	whenDataLastReceived.store(millis());				// prevent another request closing the file
	closedLoopFile.store(f);
	return true;
}

// Close the data collection file. Avoid a race between the two tasks that access it.
static void CloseDataCollectionFile() noexcept
{
	FileStore *_ecv_null const f = closedLoopFile.exchange(nullptr);
	if (f != nullptr)
	{
		f->Truncate();				// truncate the file in case we didn't write all the preallocated space
		f->Close();
		reprap.GetExpansion().AddClosedLoopRun(expectedRemoteBoardAddress, expectedRemoteSampleNumber);
	}
}

// Handle M569.5 - Collect closed loop data
GCodeResult ClosedLoop::StartDataCollection(DriverId driverId, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	// Check what the user wants - record or report
	// Record - A number of samples (Snn) is given to record
	// Report - Samples (Snn) is not given
	bool recording = false;
	uint32_t parsedS;
	gb.TryGetLimitedUIValue('S', parsedS, recording, MaxSamples + 1);
	if (!recording)
	{
		// No S parameter was given so this is a request for the recording status
		if (closedLoopFile.load() == nullptr)
		{
			reply.copy("Closed loop data is not being collected");
			return GCodeResult::warning;							// looks like the closed loop plugin relies on this being a warning
		}
		else
		{
			reply.printf("Collecting sample: %u/%" PRIu32, expectedRemoteSampleNumber, numSamplesRequested.load());
			return GCodeResult::ok;
		}
	}

	// If we get here then the user is requesting a recording
	if (closedLoopFile.load() != nullptr)							// check if one is already happening
	{
		const uint32_t wlr = whenDataLastReceived.load();			// load this volatile variable before calling millis()
		if (millis() - wlr >= DataReceiveTimeout)
		{
			CloseDataCollectionFile();								// this case is to allow us to reset if data collection stalls
			reply.copy("Closed loop data collection timed out, closing file");
		}
		else
		{
			reply.copy("Closed loop data is already being collected");
		}
		return GCodeResult::error;
	}

	// Parse the additional parameters
	bool seen = false;
	uint32_t parsedA = 0, parsedD = 0, parsedR = 0, parsedV = 0;

	gb.TryGetLimitedUIValue('A', parsedA, seen, 2);					// valid collection modes are 0 and 1
	gb.TryGetUIValue('D', parsedD, seen);
	gb.TryGetLimitedUIValue('R', parsedR, seen, std::numeric_limits<uint16_t>::max() + 1);
	gb.TryGetUIValue('V', parsedV, seen);

	// A whole sample has to fit in one CAN data message - see MaxClosedLoopSampleBytes in Duet3Common.h.
	// There are more recordable variables defined than will fit at once, so reject an over-budget request
	// here with the numbers the user needs in order to drop something. Without this the expansion board's
	// packing loop would write past the end of the CAN message payload.
	if (!ClosedLoopSampleFits(parsedD))
	{
		reply.printf("Requested variables need %u bytes per sample but the limit is %u; select fewer",
						(unsigned int)ClosedLoopSampleLength(parsedD), (unsigned int)MaxClosedLoopSampleBytes);
		return GCodeResult::error;
	}

	// Validation passed - store the values
	modeRequested = parsedA;
	rateRequested = parsedR;
	filterRequested = parsedD;
	dataBytesPerSample = ClosedLoopSampleLength(filterRequested);
	deviceRequested = driverId;
	movementRequested = parsedV;
	numSamplesRequested.store(parsedS);

	// Estimate how large the file will be
	const unsigned int numVariables = Bitmap<uint32_t>(filterRequested).CountSetBits() + 1;		// 1 extra for time stamp
	const uint32_t preallocSize = numSamplesRequested.load() * ((numVariables * 8) + 4);		// assume format "xxx.xxx," for most samples

	// Create the file
	String<StringLength50> tempFilename;
	if (gb.Seen('F'))
	{
		gb.GetQuotedString(tempFilename.GetRef(), false);
	}
	else
	{
		// Create default filename as none was provided
		const time_t timeNow = reprap.GetPlatform().GetDateTime();
		tm timeInfo;
		gmtime_r(&timeNow, &timeInfo);
		tempFilename.printf("0:/sys/closed-loop/%u_%04u-%02u-%02u_%02u.%02u.%02u.csv",
						(unsigned int) deviceRequested.boardAddress,
						timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
	}

	String<MaxFilenameLength> closedLoopFileName;
	MassStorage::CombineName(closedLoopFileName.GetRef(), "0:/sys/closed-loop/", tempFilename.c_str());
	if (!OpenDataCollectionFile(closedLoopFileName.c_str(), preallocSize))
	{
		reply.copy("failed to create data collection file");
		return GCodeResult::error;
	}

	// Set up the expected CAN address and next point number before we do anything that might call CloseDataFile
	expectedRemoteSampleNumber = 0;
	expectedRemoteBoardAddress = deviceRequested.boardAddress;

	// If no samples have been requested, return with an info message
	if (numSamplesRequested.load() == 0)
	{
		CloseDataCollectionFile();
		reply.copy("no samples recorded");
		return GCodeResult::warning;
	}

	// Set up & start the CAN data transfer
	const GCodeResult rslt = CanInterface::StartClosedLoopDataCollection(deviceRequested, filterRequested, numSamplesRequested.load(), rateRequested, movementRequested, modeRequested, gb, reply);
	if (rslt > GCodeResult::warning)
	{
		CloseDataCollectionFile();
		(void)MassStorage::Delete(closedLoopFileName.GetRef(), ErrorMessageMode::messageAlways);
	}
	return rslt;
}

// Process closed loop data received over CAN
void ClosedLoop::ProcessReceivedData(CanAddress src, const CanMessageClosedLoopData& msg, size_t msgLen) noexcept
{
	FileStore *_ecv_null const f = closedLoopFile.load();
	if (f != nullptr)
	{
		whenDataLastReceived.store(millis());
		if (msg.firstSampleNumber != expectedRemoteSampleNumber)
		{
			f->Write("Data lost\n");
			CloseDataCollectionFile();
		}
		else if (dataBytesPerSample * msg.numSamples > CanMessageClosedLoopData::GetNumDataBytes(msgLen))
		{
			f->Write("Bad data received\n");
			CloseDataCollectionFile();
		}
		else
		{
			const uint8_t *_ecv_array dataPtr = msg.data;
			for (unsigned int sampleIndex = 0; sampleIndex < msg.numSamples; ++sampleIndex)
			{
				// Compile the data
				String<StringLength256> currentLine;
				currentLine.printf("%u,%.2f", msg.firstSampleNumber + sampleIndex, (double)FetchLEF32(dataPtr));	// sample number and time stamp
				// Decoded from the same table that produced the heading line, so the columns cannot get
				// out of step with their names. Each Fetch advances dataPtr by that field's width, so the
				// iteration order here IS the wire order.
				for (const ClosedLoopChannel& chan : ClosedLoopChannels)
				{
					if (filterRequested & chan.bit)
					{
						currentLine.cat(',');
						switch (chan.kind)
						{
						case ClFieldKind::i32:	currentLine.catf(chan.format, FetchLEI32(dataPtr)); break;
						case ClFieldKind::f32:	currentLine.catf(chan.format, (double)FetchLEF32(dataPtr)); break;
						case ClFieldKind::f16:	currentLine.catf(chan.format, (double)FetchLEF16(dataPtr)); break;
						case ClFieldKind::u16:	currentLine.catf(chan.format, FetchLEU16(dataPtr)); break;
						case ClFieldKind::i16:	currentLine.catf(chan.format, FetchLEI16(dataPtr)); break;
						}
					}
				}
				currentLine.cat("\n");

				// Write the data
				f->Write(currentLine.c_str());

				// Increment the working variables
				expectedRemoteSampleNumber++;
			}

			if (msg.lastPacket)
			{
				if (msg.overflowed)
				{
					f->Write("Buffer overflowed\n");
				}
				if (msg.badSample)
				{
					f->Write("Data contains bad sample(s)\n");
				}
				CloseDataCollectionFile();
			}
		}
	}
}

#endif

// End
