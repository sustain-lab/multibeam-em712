import glob
import os
import sys
import struct
import time
import datetime
import utm
import xml.etree.ElementTree as ET
import requests
import dateutil.parser

times = []
tides = []
noTides = -1

min_e = min_n = 99.0
max_e = max_n = 0.0
start_t = stop_t = -1

lastidt = 0
outfile = 0

# Version 2 of MRZ can contain seabed image from pings of different frequencies.
# This is one way to separate them.
freqs = []
# Files used to store different seabed images from different frequencies.
freqfilewrites = []
activeOnOff=0

stdstr = ""

#Receive an array of all tokens from a seabedimage-file
#Return a string in csv-format
def makeStringFromSeabedImage(tokval):
	no = len(tokval)
	# Interpolate between positions.  First I skip the samples outside center
	# sample on each side
	prevX = ""
	prevY = ""
	thisX = ""
	thisY = ""
	prevIdx = -1
	retval = ""
	for i in range(no):
		found = tokval[i].find('(')
		if (found >= 0):
			nxt = tokval[i + 1].find(')')
			if (nxt < 0):
				return "error"
			if (len(thisY) < 1):
				prevY = tokval[i]
				prevX = tokval[i + 1]
				prevIdx = i + 2 #First valid value
				thisX = prevX
				thisY = prevY
			else:
				thisY = tokval[i]
				thisX = tokval[i + 1]
				#Then interpolate the values between prevIdx and i; i-1 being the last
				#value
				#First convert string to float, then to utm
				px = float(prevX[:-1])
				py = float(prevY[1:])
				tx = float(thisX[:-1])
				ty = float(thisY[1:])
				u = utm.from_latlon(py, px)
				x1 = u[0]
				y1 = u[1]
				u = utm.from_latlon(ty, tx)
				x2 = u[0]
				y2 = u[1]
				noSamp = i - prevIdx
				xd = float(x2 - x1) / float(noSamp)
				yd = float(y2 - y1) / float(noSamp)
				for s in range(noSamp):
					samp = float(tokval[prevIdx + s]) * 0.1
					x = x1 + xd * s
					y = y1 + yd * s
					str = "%.2f %.2f 0.0 %.1f\n" % (x,y,samp)
					retval += str
					break_me_here = 0 #debug purposes
				prevX = thisX
				prevY = thisY
				prevIdx = i + 2

#Then add the outer samples.  Mirror the second (last) position to find the
#direction and interpolation distance
#You can change the output order if necessary to get a nice output in the
#csv-file
	first = -1
	for i in range(no):
		a = tokval[i].find('(')
		if (a >= 0):
			first = i
			break

	second = -1
	for i in range(no):
		a = tokval[i].find('(')
		if (a >= 0):
			if (i != first):
				second = i
				break

	last = -1
	for i in range(no):
		a = tokval[no - 1 - i].find('(')
		if (a >= 0):
			last = i
			break

	second_last = -1
	for i in range(no):
		a = tokval[no - 1 - i].find('(')
		if (a >= 0):
			if (i != last):
				second_last = i
				break
	
	last = no - last - 1
	second_last = no - second_last - 1

	px = float(tokval[first + 1][:-1])
	py = float(tokval[first][1:])
	firstu = utm.from_latlon(py, px)
	firstx = firstu[0]
	firsty = firstu[1]
	px = float(tokval[second + 1][:-1])
	py = float(tokval[second][1:])
	secondu = utm.from_latlon(py, px)
	secondx = secondu[0]
	secondy = secondu[1]
	fxd = (secondx - firstx) / (second - first - 2)
	fyd = (secondy - firsty) / (second - first - 2)

	px = float(tokval[last + 1][:-1])
	py = float(tokval[last][1:])
	lastu = utm.from_latlon(py, px)
	lastx = lastu[0]
	lasty = lastu[1]
	px = float(tokval[second_last + 1][:-1])
	py = float(tokval[second_last][1:])
	second_lastu = utm.from_latlon(py, px)
	second_lastx = second_lastu[0]
	second_lasty = second_lastu[1]
	lxd = (second_lastx - lastx) / (second_last - last - 2)
	lyd = (second_lasty - lasty) / (second_last - last - 2)

	for i in range(no):
		if (i == first):
			break
		samp = float(tokval[i]) * 0.1
		x = firstx - i * fxd # use - as we are moving outwards from second through first and beyond
		y = firsty - i * fyd
		str = "%.2f %.2f 0.0 %.1f\n" % (x,y,samp)
		retval += str


	for i in range(no):
		k = no - 1 - i
		if (k == last + 1):
			break
		samp = float(tokval[last + i + 2]) * 0.1
		x = lastx + i * lxd
		y = lasty + i * lyd
		str = "%.2f %.2f 0.0 %.1f\n" % (x,y,samp)
		retval += str

	return retval


def openTidefile(minn, mine, maxn, maxe, mint, maxt):
	global times
	global tides
	global noTides
	
	middle_n = minn + ((maxn - minn) / 2.0)
	middle_e = mine + ((maxe - mine) / 2.0)
	fromtime = datetime.datetime.fromtimestamp(mint)
	totime = datetime.datetime.fromtimestamp(maxt)
	rfrom = fromtime.isoformat()
	ttime = totime.isoformat()
	strs = 'http://api.sehavniva.no/tideapi.php?tide_request=locationdata&lat=%.8f&lon=%.8f&datatype=OBS&lang=nl&tzone=0&refcode=CD&fromtime=%s&totime=%s&interval=10' % (middle_n, middle_e, rfrom, ttime)
	try:
		r = requests.get(strs)
		root = ET.fromstring(r.text)
		for tidelevel in root.iter('waterlevel'):
			tide = tidelevel.attrib.get('value')
			time = tidelevel.attrib.get('time')
			yourdate = dateutil.parser.parse(time)
			# Tide in SIS is negative; ADD tide to get to correct level.
			tides.append(float(tide) * -0.01)
			times.append(int(yourdate.timestamp()))
	except:
		print(strs)
		print("Failed.\n\n")
	noTides = len(times)
		

def getTide(secSinceEpoch):
	global lastidt
	global times
	global tides
	global noTides
	
	if (noTides <= 0):
		print("Cannot read tidefile")
		sys.exit(2)

	maxtime = times[noTides - 1]
	if (secSinceEpoch < times[0] or secSinceEpoch > maxtime):
		return 999999
	p = lastidt
	while (times[p] >= secSinceEpoch):
		p = p - 1
	while (times[p] <= secSinceEpoch):
		p = p + 1
	n = p - 1
	if (times[n] <= secSinceEpoch and times[p] >= secSinceEpoch):
		atime = float(times[n])
		btime = float(times[p])
		atide = float(tides[n])
		btide = float(tides[p])
		ntide = atide + (secSinceEpoch - atime) * (btide - atide) / (btime - atime)
		lastidt = n
		return ntide
	return 999999


# Process one depth datagram, #MRZ
# lengtha and chunk are from processDatagram, see below
# millisec is decoded from the header, so I send it in as a parameter here
def processDepthDatagram2(millisec, lengtha, chunk):
	global outfile
	global min_e
	global min_n
	global max_e
	global max_n
	global start_t
	global stop_t
	global stdstr
	global freqs # Frequencies used in MF-mode
	global freqfiles
	global activeOnOff
	
# Headersize is 4 bytes smaller than in the headerfile, remember that the 4
# bytes with the length has been dropped
	headersize = 1 + 1 + 1 + 1 + 1 + 1 + 2 + 4 + 4
	
	partitionsize = 2 + 2
	commonsize = 2 + 2 + 8
	common = struct.Struct('HHBBBBBBBB')
	numBytesCmnPart, pingCnt, rxFansPerPing, rxFanIndex, swathsPerPing, swathAlongPosition, \
	txTransducerInd, rxTransducerInd, numRxTransducers, algorithmType = common.unpack_from(chunk, headersize + partitionsize)
	
	pinginfo_size = 2 + 2 + 4 + 1 + 1 + 1 + 1 + 1 + 1 + 2 + 11 * 4 + 2 + 2 + 1 + 1 + 2 + 4 + 4 + 4 + 4 + 2 + 2 + 4 + 2 + 2 + 6 * 4 + 1 + 1 + 1 + 1 + 8 + 8 + 4 + 8

	pinginfo = struct.Struct('HHfBBBBBBHfffffffffffhhBBHIfffHHfHHffffffBBBBddf')

	numBytesInfoData, padding0, pingRate_Hz, beamSpacing, depthMode,\
	subDepthMode, distanceBtwSwath, detectionMode, pulseForm, \
	padding01, frequencyMode_Hz, freqRangeLowLim_Hz, \
	freqRangeHighLim_Hz, maxTotalTxPulseLength_sec, \
	maxEffTxPulseLength_sec, maxEffTxBandWidth_Hz, \
	absCoeff_dBPerkm, portSectorEdge_deg, \
	starbSectorEdge_deg, portMeanCov_deg, \
	starbMeanCov_deg, portMeanCov_m, \
	starbMeanCov_m, modeAndStabilisation, \
	runtimeFilter1, runtimeFilter2,\
	pipeTrackingStatus, transmitArraySizeUsed_deg,\
	receiveArraySizeUsed_deg, transmitPower_dB,\
	SLrampUpTimeRemaining, padding1,\
	yawAngle_deg, numTxSectors, numBytesPerTxSector,\
	headingVessel_deg, soundSpeedAtTxDepth_mPerSec,\
	txTransducerDepth_m, z_waterLevelReRefPoint_m, \
	x_txTransducerArm_SCS_m, y_txTransducerArm_SCS_m,\
	latLongInfo, posSensorStatus, attitudeSensorStatus,\
	padding2, latitude_deg, longitude_deg,\
	ellipsoidHeightReRefPoint_m = pinginfo.unpack_from(chunk, headersize + partitionsize + commonsize)
# Bug in Python, fix it (binary alignments not correct)
	latlon = struct.Struct("d")
	klat = latlon.unpack_from(chunk, headersize + partitionsize + commonsize + 124)
	klon = latlon.unpack_from(chunk, headersize + partitionsize + commonsize + 124 + 8)
	ellheight = struct.Struct("f")
	ellipsheight = ellheight.unpack_from(chunk, headersize + partitionsize + commonsize + 124 + 8 + 8)
	latitude_deg = klat[0]
	longitude_deg = klon[0]
	ellipsoidHeightReRefPoint_m = ellipsheight[0]
	# Changed in Version 1
	bsCorrectionOffset_dB = ellheight.unpack_from(chunk, headersize + partitionsize + commonsize + 124 + 8 + 8 + 4)[0]
	byterec = struct.Struct("B")
	lambertsLawApplied = byterec.unpack_from(chunk, headersize + partitionsize + commonsize + 124 + 8 + 8 + 4 + 4)[0]
	iceWindow = byterec.unpack_from(chunk, headersize + partitionsize + commonsize + 124 + 8 + 8 + 4 + 4 + 1)[0]
	shortrec = struct.Struct("H")
    # The paddig is in version 2 used for activeModes.
    # Remember that this is just an indication that MF is in use; we must look at the
    # actual frequencyMode_Hz used and the pulsetype to determine which seabed image to use
	activeModes = shortrec.unpack_from(chunk, headersize + partitionsize + commonsize + 124 + 8 + 8 + 4 + 4 + 1 + 1)[0]
	
	sec = int(millisec / 1000)
	
# Pointer offset to sectorInfo
	sectorInfo_offset = headersize + partitionsize + commonsize + pinginfo_size
	# Changed from version 0
	sectorInfo = struct.Struct('BBBBfffffffBBHfff')
	sectorInfo_size = 1 + 1 + 1 + 1 + 7 * 4 + 1 + 1 + 2 + 4 + 4 + 4
	i = 0
	while (i < numTxSectors):
		txSectorNumb, txArrNumber, txSubArray, padding0,\
		sectorTransmitDelay_sec, tiltAngleReTx_deg,\
		txNominalSourceLevel_dB, txFocusRange_m,\
		centreFreq_Hz, signalBandWidth_Hz, \
		totalSignalLength_sec, pulseShading, signalWaveForm,\
		padding1, highVoltageLevel_dB, sectorTrackingCorr_dB, effectiveSignalLength_sec = sectorInfo.unpack_from(chunk, sectorInfo_offset + i * sectorInfo_size)
		i+=1

	rxInfo_offset = sectorInfo_offset + numTxSectors * sectorInfo_size
	rxInfo = struct.Struct('HHHHffffHHHH')
	rxInfo_size = 2 + 2 + 2 + 2 + 4 + 4 + 4 + 4 + 2 + 2 + 2 + 2
	numBytesRxInfo, numSoundingsMaxMain, numSoundingsValidMain, numBytesPerSounding, \
	WCSampleRate, seabedImageSampleRate, BSnormal_dB, BSoblique_dB, \
	extraDetectionAlarmFlag, numExtraDetections, numExtraDetectionClasses, \
	numBytesPerClass = rxInfo.unpack_from(chunk, rxInfo_offset)
	
	extraDetClassInfo_offset = rxInfo_offset + rxInfo_size

	extraDetectionSize = 2 + 1 + 1
	extraDetectionStruct = struct.Struct('HBB')
		
	sounding_offset = extraDetClassInfo_offset + numExtraDetectionClasses * extraDetectionSize

	soundingStruct = struct.Struct('HBBBBBBBBHffffffHHffffffffffffffffffHHHH')
	sounding_size = 2 + 8 + 2 + 6 * 4 + 2 + 2 + 18 * 4 + 4 * 2

    #Offset to seabed image
	seabedImageStart = sounding_offset + (sounding_size * (numSoundingsMaxMain + numExtraDetections))
	seabedStruct = struct.Struct('h')
	sbed_len = lengtha + 4 - seabedImageStart - 4
	tot_no_sbed = sbed_len / 2
	verify_length = tot_no_sbed * 2
	lenStruct = struct.Struct('I')
	dgmlenver = seabedImageStart + sbed_len
	dgmlen = lenStruct.unpack_from(chunk,dgmlenver - 4)[0] # should be 4 more then lengtha

	outputstr = "\n%.8f %.8f %.2f %.2f %d\n" % (latitude_deg, longitude_deg, 
		ellipsoidHeightReRefPoint_m, z_waterLevelReRefPoint_m, millisec)
	outfile.write(outputstr)
	sbed_start = seabedImageStart # This is the pointer to the start of the seabed image for current beam
	no_sbed_found = 0
	i = 0
	stdstr = ""
	while(i < numSoundingsMaxMain):
		soundingIndex, txSectorNumb, detectionType, \
		detectionMethod, rejectionInfo1, rejectionInfo2, \
		postProcessingInfo, detectionClass, detectionConfidenceLevel, \
		padding, rangeFactor, qualityFactor, \
		detectionUncertaintyVer_m, detectionUncertaintyHor_m, \
		detectionWindowLength_sec, echoLength_sec, \
		WCBeamNumb, WCrange_samples, WCNomBeamAngleAcross_deg, \
		meanAbsCoeff_dBPerkm, reflectivity1_dB, reflectivity2_dB, \
		receiverSensitivityApplied_dB, sourceLevelApplied_dB, \
		BScalibration_dB, TVG_dB, beamAngleReRx_deg, \
		beamAngleCorrection_deg, twoWayTravelTime_sec, \
		twoWayTravelTimeCorrection_sec, deltaLatitude_deg, \
		deltaLongitude_deg, z_reRefPoint_m, y_reRefPoint_m, \
		x_reRefPoint_m, beamIncAngleAdj_deg, realTimeCleanInfo, \
		SIstartRange_samples, SIcentreSample, \
		SInumSamples = soundingStruct.unpack_from(chunk, sounding_offset + i * sounding_size)
		i+=1
			
# THIS IS IT.  This is where we output xyz-points
# Depths are referred to the reference point.  To get it to the waterline,
# SUBSTRACT the distance from
# Error estimates are also available: detectionUncertaintyVer_m and
# detectionUncertaintyHor_m
		waterlevel = z_reRefPoint_m - z_waterLevelReRefPoint_m
		plat = latitude_deg + deltaLatitude_deg
		plon = longitude_deg + deltaLongitude_deg
		outputstr = " %.8f %.8f %.2f %.2f %.2f" % (deltaLatitude_deg, deltaLongitude_deg, 
			z_reRefPoint_m, detectionUncertaintyVer_m, detectionUncertaintyHor_m)
		outfile.write(outputstr)
		n = float(latitude_deg)
		e = float(longitude_deg)
		t = int(millisec)
		if (start_t < 0 or t < start_t):
			start_t = t
		if (t > stop_t):
			stop_t = t
		if (min_e > e):
			min_e = e
		if (min_n > n):
			min_n = n
		if (e > max_e):
			max_e = e
		if (n > max_n):
			max_n = n
			
		next_sbd_start = sbed_start + (2 * SInumSamples)
		if (y_reRefPoint_m < 0): # Reverse the output of the samples, see documentation
			sbed_start = next_sbd_start - 2
			center_samp = SInumSamples - SIcentreSample
		else:
			center_samp = SIcentreSample

		for n in range(0, SInumSamples):
			no_sbed_found += 1
			if (n == center_samp): # Put in position of center sample
				outputstr = " (%.8f %.8f) " % (plat, plon)
				str11 = stdstr + outputstr
				stdstr = str11
			sbed_sample = seabedStruct.unpack_from(chunk, sbed_start)[0]
			outputstr = " %d" % (sbed_sample)
			str11 = stdstr + outputstr
			stdstr = str11
			if (y_reRefPoint_m < 0):
				sbed_start -= 2
			else:
				sbed_start += 2 # jump 2 bytes (short) forwards

		sbed_start = next_sbd_start
		# There are 9 samples per extra detection, and there may be 2 bytes padding
		# at the end
		if (i > 398):
			break_me_here = 0 # for debugging purposes

	snstr = makeStringFromSeabedImage(stdstr.split())
	outw = outfileSBD
	if (activeModes == 1):
		middlefreq = int(frequencyMode_Hz)#Get rid of commas
		# Select file to open and write to		
		if (activeOnOff == 0):
			activeOnOff = 1
			print("Active mode found")
		found = 0
		fcnt = 0
		for fmd in freqs:
			fcnt = fcnt + 1
			if (fmd == middlefreq):
				found = 1
				outw = freqfilewrites[fcnt - 1]
		
		if (found == 0): # New entry
			fname = file + "_"+str(middlefreq)+".seabed.csv"
			freqs.append(middlefreq)
			freqfilewrites.append(open(fname,'w', encoding='utf-8'))
			outw = freqfilewrites[len(freqfilewrites)-1]
	
	outw.write(snstr)	
	#outfileSBD.write(snstr)


# Process one depth datagram, #MRZ
# lengtha and chunk are from processDatagram, see below
# millisec is decoded from the header, so I send it in as a parameter here
def processDepthDatagram1(millisec, lengtha, chunk):
	global outfile
	global min_e
	global min_n
	global max_e
	global max_n
	global start_t
	global stop_t
	global stdstr
	
# Headersize is 4 bytes smaller than in the headerfile, remember that the 4
# bytes with the length has been dropped
	headersize = 1 + 1 + 1 + 1 + 1 + 1 + 2 + 4 + 4
	
	partitionsize = 2 + 2
	commonsize = 2 + 2 + 8
	common = struct.Struct('HHBBBBBBBB')
	numBytesCmnPart, pingCnt, rxFansPerPing, rxFanIndex, swathsPerPing, swathAlongPosition, \
	txTransducerInd, rxTransducerInd, numRxTransducers, algorithmType = common.unpack_from(chunk, headersize + partitionsize)
	
	pinginfo_size = 2 + 2 + 4 + 1 + 1 + 1 + 1 + 1 + 1 + 2 + 11 * 4 + 2 + 2 + 1 + 1 + 2 + 4 + 4 + 4 + 4 + 2 + 2 + 4 + 2 + 2 + 6 * 4 + 1 + 1 + 1 + 1 + 8 + 8 + 4 + 8

	pinginfo = struct.Struct('HHfBBBBBBHfffffffffffhhBBHIfffHHfHHffffffBBBBddf')

	numBytesInfoData, padding0, pingRate_Hz, beamSpacing, depthMode,\
	subDepthMode, distanceBtwSwath, detectionMode, pulseForm, \
	padding01, frequencyMode_Hz, freqRangeLowLim_Hz, \
	freqRangeHighLim_Hz, maxTotalTxPulseLength_sec, \
	maxEffTxPulseLength_sec, maxEffTxBandWidth_Hz, \
	absCoeff_dBPerkm, portSectorEdge_deg, \
	starbSectorEdge_deg, portMeanCov_deg, \
	starbMeanCov_deg, portMeanCov_m, \
	starbMeanCov_m, modeAndStabilisation, \
	runtimeFilter1, runtimeFilter2,\
	pipeTrackingStatus, transmitArraySizeUsed_deg,\
	receiveArraySizeUsed_deg, transmitPower_dB,\
	SLrampUpTimeRemaining, padding1,\
	yawAngle_deg, numTxSectors, numBytesPerTxSector,\
	headingVessel_deg, soundSpeedAtTxDepth_mPerSec,\
	txTransducerDepth_m, z_waterLevelReRefPoint_m, \
	x_txTransducerArm_SCS_m, y_txTransducerArm_SCS_m,\
	latLongInfo, posSensorStatus, attitudeSensorStatus,\
	padding2, latitude_deg, longitude_deg,\
	ellipsoidHeightReRefPoint_m = pinginfo.unpack_from(chunk, headersize + partitionsize + commonsize)
# Bug in Python, fix it (binary alignments not correct)
	latlon = struct.Struct("d")
	klat = latlon.unpack_from(chunk, headersize + partitionsize + commonsize + 124)
	klon = latlon.unpack_from(chunk, headersize + partitionsize + commonsize + 124 + 8)
	ellheight = struct.Struct("f")
	ellipsheight = ellheight.unpack_from(chunk, headersize + partitionsize + commonsize + 124 + 8 + 8)
	latitude_deg = klat[0]
	longitude_deg = klon[0]
	ellipsoidHeightReRefPoint_m = ellipsheight[0]
	# Changed in Version 1
	bsCorrectionOffset_dB = ellheight.unpack_from(chunk, headersize + partitionsize + commonsize + 124 + 8 + 8 + 4)[0]
	byterec = struct.Struct("B")
	lambertsLawApplied = byterec.unpack_from(chunk, headersize + partitionsize + commonsize + 124 + 8 + 8 + 4 + 4)[0]
	iceWindow = byterec.unpack_from(chunk, headersize + partitionsize + commonsize + 124 + 8 + 8 + 4 + 4 + 1)[0]
	shortrec = struct.Struct("H")
	padding4 = shortrec.unpack_from(chunk, headersize + partitionsize + commonsize + 124 + 8 + 8 + 4 + 4 + 1 + 1)[0]
	
	sec = int(millisec / 1000)
	
# Pointer offset to sectorInfo
	sectorInfo_offset = headersize + partitionsize + commonsize + pinginfo_size
	# Changed from version 0
	sectorInfo = struct.Struct('BBBBfffffffBBHfff')
	sectorInfo_size = 1 + 1 + 1 + 1 + 7 * 4 + 1 + 1 + 2 + 4 + 4 + 4
	i = 0
	while (i < numTxSectors):
		txSectorNumb, txArrNumber, txSubArray, padding0,\
		sectorTransmitDelay_sec, tiltAngleReTx_deg,\
		txNominalSourceLevel_dB, txFocusRange_m,\
		centreFreq_Hz, signalBandWidth_Hz, \
		totalSignalLength_sec, pulseShading, signalWaveForm,\
		padding1, highVoltageLevel_dB, sectorTrackingCorr_dB, effectiveSignalLength_sec = sectorInfo.unpack_from(chunk, sectorInfo_offset + i * sectorInfo_size)
		i+=1

	rxInfo_offset = sectorInfo_offset + numTxSectors * sectorInfo_size
	rxInfo = struct.Struct('HHHHffffHHHH')
	rxInfo_size = 2 + 2 + 2 + 2 + 4 + 4 + 4 + 4 + 2 + 2 + 2 + 2
	numBytesRxInfo, numSoundingsMaxMain, numSoundingsValidMain, numBytesPerSounding, \
	WCSampleRate, seabedImageSampleRate, BSnormal_dB, BSoblique_dB, \
	extraDetectionAlarmFlag, numExtraDetections, numExtraDetectionClasses, \
	numBytesPerClass = rxInfo.unpack_from(chunk, rxInfo_offset)
	
	extraDetClassInfo_offset = rxInfo_offset + rxInfo_size

	extraDetectionSize = 2 + 1 + 1
	extraDetectionStruct = struct.Struct('HBB')
		
	sounding_offset = extraDetClassInfo_offset + numExtraDetectionClasses * extraDetectionSize

	soundingStruct = struct.Struct('HBBBBBBBBHffffffHHffffffffffffffffffHHHH')
	sounding_size = 2 + 8 + 2 + 6 * 4 + 2 + 2 + 18 * 4 + 4 * 2

    #Offset to seabed image
	seabedImageStart = sounding_offset + (sounding_size * (numSoundingsMaxMain + numExtraDetections))
	seabedStruct = struct.Struct('h')
	sbed_len = lengtha + 4 - seabedImageStart - 4
	tot_no_sbed = sbed_len / 2
	verify_length = tot_no_sbed * 2
	lenStruct = struct.Struct('I')
	dgmlenver = seabedImageStart + sbed_len
	dgmlen = lenStruct.unpack_from(chunk,dgmlenver - 4)[0] # should be 4 more then lengtha

	outputstr = "\n%.8f %.8f %.2f %.2f %d\n" % (latitude_deg, longitude_deg, 
		ellipsoidHeightReRefPoint_m, z_waterLevelReRefPoint_m, millisec)
	outfile.write(outputstr)
	sbed_start = seabedImageStart # This is the pointer to the start of the seabed image for current beam
	no_sbed_found = 0
	i = 0
	stdstr = ""
	while(i < numSoundingsMaxMain):
		soundingIndex, txSectorNumb, detectionType, \
		detectionMethod, rejectionInfo1, rejectionInfo2, \
		postProcessingInfo, detectionClass, detectionConfidenceLevel, \
		padding, rangeFactor, qualityFactor, \
		detectionUncertaintyVer_m, detectionUncertaintyHor_m, \
		detectionWindowLength_sec, echoLength_sec, \
		WCBeamNumb, WCrange_samples, WCNomBeamAngleAcross_deg, \
		meanAbsCoeff_dBPerkm, reflectivity1_dB, reflectivity2_dB, \
		receiverSensitivityApplied_dB, sourceLevelApplied_dB, \
		BScalibration_dB, TVG_dB, beamAngleReRx_deg, \
		beamAngleCorrection_deg, twoWayTravelTime_sec, \
		twoWayTravelTimeCorrection_sec, deltaLatitude_deg, \
		deltaLongitude_deg, z_reRefPoint_m, y_reRefPoint_m, \
		x_reRefPoint_m, beamIncAngleAdj_deg, realTimeCleanInfo, \
		SIstartRange_samples, SIcentreSample, \
		SInumSamples = soundingStruct.unpack_from(chunk, sounding_offset + i * sounding_size)
		i+=1
			
# THIS IS IT.  This is where we output xyz-points
# Depths are referred to the reference point.  To get it to the waterline,
# SUBSTRACT the distance from
# Error estimates are also available: detectionUncertaintyVer_m and
# detectionUncertaintyHor_m
		waterlevel = z_reRefPoint_m - z_waterLevelReRefPoint_m
		plat = latitude_deg + deltaLatitude_deg
		plon = longitude_deg + deltaLongitude_deg
		outputstr = " %.8f %.8f %.2f %.2f %.2f" % (deltaLatitude_deg, deltaLongitude_deg, 
			z_reRefPoint_m, detectionUncertaintyVer_m, detectionUncertaintyHor_m)
		outfile.write(outputstr)
		n = float(latitude_deg)
		e = float(longitude_deg)
		t = int(millisec)
		if (start_t < 0 or t < start_t):
			start_t = t
		if (t > stop_t):
			stop_t = t
		if (min_e > e):
			min_e = e
		if (min_n > n):
			min_n = n
		if (e > max_e):
			max_e = e
		if (n > max_n):
			max_n = n
			
		next_sbd_start = sbed_start + (2 * SInumSamples)
		if (y_reRefPoint_m < 0): # Reverse the output of the samples, see documentation
			sbed_start = next_sbd_start - 2
			center_samp = SInumSamples - SIcentreSample
		else:
			center_samp = SIcentreSample

		for n in range(0, SInumSamples):
			no_sbed_found += 1
			if (n == center_samp): # Put in position of center sample
				outputstr = " (%.8f %.8f) " % (plat, plon)
				str11 = stdstr + outputstr
				stdstr = str11
			sbed_sample = seabedStruct.unpack_from(chunk, sbed_start)[0]
			outputstr = " %d" % (sbed_sample)
			str11 = stdstr + outputstr
			stdstr = str11
			if (y_reRefPoint_m < 0):
				sbed_start -= 2
			else:
				sbed_start += 2 # jump 2 bytes (short) forwards

		sbed_start = next_sbd_start
		# There are 9 samples per extra detection, and there may be 2 bytes padding
		# at the end
		if (i > 398):
			break_me_here = 0 # for debugging purposes

	snstr = makeStringFromSeabedImage(stdstr.split())
	outfileSBD.write(snstr)


# Process one depth datagram, #MRZ
# lengtha and chunk are from processDatagram, see below
# millisec is decoded from the header, so I send it in as a parameter here
def processDepthDatagram(millisec, lengtha, chunk):
	global outfile
	global min_e
	global min_n
	global max_e
	global max_n
	global start_t
	global stop_t
	global stdstr
	
# Headersize is 4 bytes smaller than in the headerfile, remember that the 4
# bytes with the length has been dropped
	headersize = 1 + 1 + 1 + 1 + 1 + 1 + 2 + 4 + 4
	
	partitionsize = 2 + 2
	commonsize = 2 + 2 + 8
	common = struct.Struct('HHBBBBBBBB')
	numBytesCmnPart, pingCnt, rxFansPerPing, rxFanIndex, swathsPerPing, swathAlongPosition, \
	txTransducerInd, rxTransducerInd, numRxTransducers, algorithmType = common.unpack_from(chunk, headersize + partitionsize)
	
	pinginfo_size = 2 + 2 + 4 + 1 + 1 + 1 + 1 + 1 + 1 + 2 + 11 * 4 + 2 + 2 + 1 + 1 + 2 + 4 + 4 + 4 + 4 + 2 + 2 + 4 + 2 + 2 + 6 * 4 + 1 + 1 + 1 + 1 + 8 + 8 + 4

	pinginfo = struct.Struct('HHfBBBBBBHfffffffffffhhBBHIfffHHfHHffffffBBBBddf')

	numBytesInfoData, padding0, pingRate_Hz, beamSpacing, depthMode,\
	subDepthMode, distanceBtwSwath, detectionMode, pulseForm, \
	padding01, frequencyMode_Hz, freqRangeLowLim_Hz, \
	freqRangeHighLim_Hz, maxTotalTxPulseLength_sec, \
	maxEffTxPulseLength_sec, maxEffTxBandWidth_Hz, \
	absCoeff_dBPerkm, portSectorEdge_deg, \
	starbSectorEdge_deg, portMeanCov_deg, \
	starbMeanCov_deg, portMeanCov_m, \
	starbMeanCov_m, modeAndStabilisation, \
	runtimeFilter1, runtimeFilter2,\
	pipeTrackingStatus, transmitArraySizeUsed_deg,\
	receiveArraySizeUsed_deg, transmitPower_dB,\
	SLrampUpTimeRemaining, padding1,\
	yawAngle_deg, numTxSectors, numBytesPerTxSector,\
	headingVessel_deg, soundSpeedAtTxDepth_mPerSec,\
	txTransducerDepth_m, z_waterLevelReRefPoint_m, \
	x_txTransducerArm_SCS_m, y_txTransducerArm_SCS_m,\
	latLongInfo, posSensorStatus, attitudeSensorStatus,\
	padding2, latitude_deg, longitude_deg,\
	ellipsoidHeightReRefPoint_m = pinginfo.unpack_from(chunk, headersize + partitionsize + commonsize)
# Bug in Python, fix it (binary alignments not correct)
	latlon = struct.Struct("d")
	klat = latlon.unpack_from(chunk, headersize + partitionsize + commonsize + 124)
	klon = latlon.unpack_from(chunk, headersize + partitionsize + commonsize + 124 + 8)
	ellheight = struct.Struct("f")
	ellipsheight = ellheight.unpack_from(chunk, headersize + partitionsize + commonsize + 124 + 8 + 8)
	latitude_deg = klat[0]
	longitude_deg = klon[0]
	ellipsoidHeightReRefPoint_m = ellipsheight[0]
	
	sec = int(millisec / 1000)
	
# Pointer offset to sectorInfo
	sectorInfo_offset = headersize + partitionsize + commonsize + pinginfo_size
	sectorInfo = struct.Struct('BBBBfffffffBBH')
	sectorInfo_size = 1 + 1 + 1 + 1 + 7 * 4 + 1 + 1 + 2
	i = 0
	while (i < numTxSectors):
		txSectorNumb, txArrNumber, txSubArray, padding0,\
		sectorTransmitDelay_sec, tiltAngleReTx_deg,\
		txNominalSourceLevel_dB, txFocusRange_m,\
		centreFreq_Hz, signalBandWidth_Hz, \
		totalSignalLength_sec, pulseShading, signalWaveForm,\
		padding1 = sectorInfo.unpack_from(chunk, sectorInfo_offset + i * sectorInfo_size)
		i+=1

	rxInfo_offset = sectorInfo_offset + numTxSectors * sectorInfo_size
	rxInfo = struct.Struct('HHHHffffHHHH')
	rxInfo_size = 2 + 2 + 2 + 2 + 4 + 4 + 4 + 4 + 2 + 2 + 2 + 2
	numBytesRxInfo, numSoundingsMaxMain, numSoundingsValidMain, numBytesPerSounding, \
	WCSampleRate, seabedImageSampleRate, BSnormal_dB, BSoblique_dB, \
	extraDetectionAlarmFlag, numExtraDetections, numExtraDetectionClasses, \
	numBytesPerClass = rxInfo.unpack_from(chunk, rxInfo_offset)
	
	extraDetClassInfo_offset = rxInfo_offset + rxInfo_size

	extraDetectionSize = 2 + 1 + 1
	extraDetectionStruct = struct.Struct('HBB')
		
	sounding_offset = extraDetClassInfo_offset + numExtraDetectionClasses * extraDetectionSize

	soundingStruct = struct.Struct('HBBBBBBBBHffffffHHffffffffffffffffffHHHH')
	sounding_size = 2 + 8 + 2 + 6 * 4 + 2 + 2 + 18 * 4 + 4 * 2

    #Offset to seabed image
	seabedImageStart = sounding_offset + (sounding_size * (numSoundingsMaxMain + numExtraDetections))
	seabedStruct = struct.Struct('h')
	sbed_len = lengtha + 4 - seabedImageStart - 4
	tot_no_sbed = sbed_len / 2
	verify_length = tot_no_sbed * 2
	lenStruct = struct.Struct('I')
	dgmlenver = seabedImageStart + sbed_len
	dgmlen = lenStruct.unpack_from(chunk,dgmlenver - 4)[0] # should be 4 more then lengtha

	outputstr = "\n%.8f %.8f %.2f %.2f %d\n" % (latitude_deg, longitude_deg, 
		ellipsoidHeightReRefPoint_m, z_waterLevelReRefPoint_m, millisec)
	outfile.write(outputstr)
	sbed_start = seabedImageStart # This is the pointer to the start of the seabed image for current beam
	no_sbed_found = 0
	i = 0
	stdstr = ""
	while(i < numSoundingsMaxMain):
		soundingIndex, txSectorNumb, detectionType, \
		detectionMethod, rejectionInfo1, rejectionInfo2, \
		postProcessingInfo, detectionClass, detectionConfidenceLevel, \
		padding, rangeFactor, qualityFactor, \
		detectionUncertaintyVer_m, detectionUncertaintyHor_m, \
		detectionWindowLength_sec, echoLength_sec, \
		WCBeamNumb, WCrange_samples, WCNomBeamAngleAcross_deg, \
		meanAbsCoeff_dBPerkm, reflectivity1_dB, reflectivity2_dB, \
		receiverSensitivityApplied_dB, sourceLevelApplied_dB, \
		BScalibration_dB, TVG_dB, beamAngleReRx_deg, \
		beamAngleCorrection_deg, twoWayTravelTime_sec, \
		twoWayTravelTimeCorrection_sec, deltaLatitude_deg, \
		deltaLongitude_deg, z_reRefPoint_m, y_reRefPoint_m, \
		x_reRefPoint_m, beamIncAngleAdj_deg, realTimeCleanInfo, \
		SIstartRange_samples, SIcentreSample, \
		SInumSamples = soundingStruct.unpack_from(chunk, sounding_offset + i * sounding_size)
		i+=1
			
# THIS IS IT.  This is where we output xyz-points
# Depths are referred to the reference point.  To get it to the waterline,
# SUBSTRACT the distance from
# Error estimates are also available: detectionUncertaintyVer_m and
# detectionUncertaintyHor_m
		waterlevel = z_reRefPoint_m - z_waterLevelReRefPoint_m
		plat = latitude_deg + deltaLatitude_deg
		plon = longitude_deg + deltaLongitude_deg
		outputstr = " %.8f %.8f %.2f %.2f %.2f" % (deltaLatitude_deg, deltaLongitude_deg, 
			z_reRefPoint_m, detectionUncertaintyVer_m, detectionUncertaintyHor_m)
		outfile.write(outputstr)
		n = float(latitude_deg)
		e = float(longitude_deg)
		t = int(millisec)
		if (start_t < 0 or t < start_t):
			start_t = t
		if (t > stop_t):
			stop_t = t
		if (min_e > e):
			min_e = e
		if (min_n > n):
			min_n = n
		if (e > max_e):
			max_e = e
		if (n > max_n):
			max_n = n
			
		next_sbd_start = sbed_start + (2 * SInumSamples)
		if (y_reRefPoint_m < 0): # Reverse the output of the samples, see documentation
			sbed_start = next_sbd_start - 2
			center_samp = SInumSamples - SIcentreSample
		else:
			center_samp = SIcentreSample

		for n in range(0, SInumSamples):
			no_sbed_found += 1
			if (n == center_samp): # Put in position of center sample
				outputstr = " (%.8f %.8f) " % (plat, plon)
				str11 = stdstr + outputstr
				stdstr = str11
			sbed_sample = seabedStruct.unpack_from(chunk, sbed_start)[0]
			outputstr = " %d" % (sbed_sample)
			str11 = stdstr + outputstr
			stdstr = str11
			if (y_reRefPoint_m < 0):
				sbed_start -= 2
			else:
				sbed_start += 2 # jump 2 bytes (short) forwards

		sbed_start = next_sbd_start
		# There are 9 samples per extra detection, and there may be 2 bytes padding
		# at the end
		if (i > 398):
			break_me_here = 0 # for debugging purposes

	snstr = makeStringFromSeabedImage(stdstr.split())
	outfileSBD.write(snstr)
			

# What happens in processDatagram?  Read the documentation of the kmall-format.
# This is the processing of the datagram to find out what datagram type this
# is.
# The processing of each datagram type takes place in specific routines
def processDatagram(lengtha, chunk):
	header_without_length = struct.Struct('ccccBBHII')
	dgm_type0,dgm_type1,dgm_type2,dgm_type3,dgm_version,sysid,emid,sec,nsec = header_without_length.unpack_from(chunk,0)
	dgm_type = dgm_type0 + dgm_type1 + dgm_type2 + dgm_type3
		
# Decode time
	nanosec = sec
	nanosec *= 1E9
	nanosec += nsec
	millisec = nanosec
	millisec /= 1E6	
	
	strk = dgm_type.decode()
	if (strk == '#MRZ'):
		if (dgm_version == 0):
			processDepthDatagram(millisec, lengtha, chunk)
		if (dgm_version == 1):
			processDepthDatagram1(millisec, lengtha, chunk)
		if (dgm_version == 2):
			processDepthDatagram2(millisec, lengtha, chunk)

# I shall not humiliate any developer by documenting this main program.
# The processing of the datagram takes place in the routine processDatagram.
files = glob.glob('*.kmall')
for file in files:
	try:
		activeOnOff = 0
		f = open(file, 'rb')
		nfile = file + ".pings"
		outfile = open(nfile,'w', encoding='utf-8')
		nfile = file + ".seabed.csv"
		outfileSBD = open(nfile,'w', encoding='utf-8')
		freqs = []
		for g in freqfilewrites:
			g.close()
		freqfilewrites = []
	except Exception:
		print('File',file,'not opened.')
		sys.exit(0)
	
	print(file)
# Process the file:
	f.seek(0, 2)
	file_size = f.tell()
	f.seek(0, 0)
	remaining = file_size

# Read all datagrams and process each of them
	while (remaining > 0):
# First read 4 bytes that contains the length of the chunk
		lengthb = struct.unpack("I",f.read(4))
		remaining -= 4
# Then read the chunk.  Note that the length read includes the 4 bytes in the
# integer.
		dgmsize = lengthb[0] - 4
		chunk = f.read(dgmsize)
		remaining -= dgmsize
# Then process this chunk
		processDatagram(dgmsize, chunk)
	
	f.close()
	outfile.close()
	outfileSBD.close()
	
# Remove empty seabed image files
filelist = glob.glob('*.seabed.csv')
for file in filelist:
	if (os.stat(file).st_size == 0):
		os.remove(file)

aatime = int(start_t / 1e3) - (24 * 60 * 60)
bbtime = int(stop_t / 1e3) + (24 * 60 * 60)
openTidefile(min_n, min_e, max_n, max_e, aatime, bbtime)

filelist = glob.glob('*.pings')
for file in filelist:
	try:
		f = open(file)
		nfile = file + ".tidecorrected.utm.csv"
		outfile = open(nfile,'w', encoding='utf-8')
	except Exception:
		print('File',file,'not opened.')
		sys.exit(0)

	print(file)
	if (len(times) <= 0):
		print("no tides, depths not corrected for tide")
	
	line = f.readline()
	while (line):
		toks = line.split()
		if (len(toks) == 5):
			n = float(toks[0])
			e = float(toks[1])
			tm = int(int(toks[4]) / 1e3)
			tide = getTide(tm)
			if (tide > 9999):
				tide = 0
			toWlev = float(toks[3]) + tide
		else:
			cnt = 0
			while(cnt < len(toks)):
				lat = n + float(toks[cnt])
				lon = e + float(toks[cnt + 1])
				dpt = float(toks[cnt + 2]) + toWlev
				u = utm.from_latlon(lat, lon)
				outputstr = "%.2f %.2f %.2f\n" % (u[0], u[1], dpt * -1.0)
				outfile.write(outputstr)			
				cnt += 5
		line = f.readline()
		
	f.close()

