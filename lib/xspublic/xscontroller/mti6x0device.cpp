
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#include "mti6x0device.h"

#include <xstypes/xsdatapacket.h>
#include "replyobject.h"
#include "communicator.h"
#include "scenariomatchpred.h"
#include <xstypes/xsstatusflag.h>

#include <xstypes/xssensorranges.h>
#include "synclinegmt.h"
#include <xstypes/xssyncsetting.h>

using namespace xsens;

/*! \brief Constructs a device
	\param comm The communicator to construct with
*/
Mti6X0Device::Mti6X0Device(Communicator* comm) :
	MtiBaseDeviceEx(comm)
{
}

/*! \brief Destroys a device
*/
Mti6X0Device::~Mti6X0Device()
{
}

/*! \brief Returns the base update rate (Hz) corresponding to the dataType
*/
MtiBaseDevice::BaseFrequencyResult Mti6X0Device::getBaseFrequencyInternal(XsDataIdentifier dataType) const
{
	MtiBaseDevice::BaseFrequencyResult result;
	result.m_frequency = 0;
	result.m_divedable = true;

	if ((dataType == XDI_FreeAcceleration && deviceId().isImu()) ||
		((dataType & XDI_FullTypeMask) == XDI_LocationId) ||
		((dataType & XDI_FullTypeMask) == XDI_DeviceId))
		return result;

	if ((dataType & XDI_FullTypeMask) == XDI_AccelerationHR || (dataType & XDI_FullTypeMask) == XDI_RateOfTurnHR)
	{
		bool isMtMk4_1_v2 = hardwareVersion().major() == 2;
		result.m_frequency = isMtMk4_1_v2 ? 800 : 1000;
		result.m_divedable = isMtMk4_1_v2 ? true : false;

		return result;
	}

	auto baseFreq = [&](XsDataIdentifier dataType)
	{
		switch (dataType & XDI_TypeMask)
		{
		case XDI_None:					return 400;
		case XDI_TimestampGroup:		return XDI_MAX_FREQUENCY_VAL;
		case XDI_StatusGroup:			return 400;
		case XDI_TemperatureGroup:		return 400;
		case XDI_OrientationGroup:		return deviceId().isImu() ? 0 : 400;
		case XDI_AccelerationGroup:		return 400;
		case XDI_AngularVelocityGroup:	return 400;
		case XDI_MagneticGroup:			return 400;

		case XDI_GnssGroup:				return deviceId().isGnss() ? 4 : 0;
		case XDI_PressureGroup:			return deviceId().isGnss() ? 50 : 0;
		case XDI_PositionGroup:			return deviceId().isGnss() ? 400 : 0;
		case XDI_VelocityGroup:			return deviceId().isGnss() ? 400 : 0;
		default:						return 0;
		}
	};
	result.m_frequency = baseFreq(dataType);

	if ((dataType & XDI_TypeMask) == XDI_TimestampGroup)
		result.m_divedable = false;

	return result;
}

/*! \returns The components information from the device.
*/
XsByteArray Mti6X0Device::componentsInformation() const
{
	uint8_t bid = busId();
	if (bid == XS_BID_INVALID || bid == XS_BID_BROADCAST)
		return XsByteArray();

	XsMessage snd(XMID_ReqComponentsInformation, 0), rcv;
	snd.setBusId(bid);

	if (!doTransaction(snd, rcv))
		return XsByteArray();

	XsByteArray componentsInfo(const_cast<uint8_t*>(rcv.getDataBuffer()), rcv.getDataSize(), XSDF_None);

	return componentsInfo;
}

/*! \returns The sync settings line for a mtmk4_x device. This overrides the base class method.
	\param buff The pointer to a buffer to get a sync setting line from
	\param offset The offeset to set for the buffer
*/
XsSyncLine Mti6X0Device::syncSettingsLine(const uint8_t* buff, XsSize offset) const
{
	return	xslgmtToXsl((SyncLineGmt) buff[offset + 1]);
}

/*! \returns The sync line for a mtmk4_x device. This overrides the base class method.
	\param setting The sync setting to get a sync line from
*/
uint8_t Mti6X0Device::syncLine(const XsSyncSetting& setting) const
{
	SyncLineGmt gmtLine = xslToXslgmt(setting.m_line);
	assert(gmtLine != XSLGMT_Invalid);
	if (gmtLine == XSLGMT_ClockIn)
	{
		assert(setting.m_function == XSF_SampleAndSend);
	}
	return static_cast<uint8_t>(gmtLine);
}

/*! \returns True if this device has an ICC support
*/
bool Mti6X0Device::hasIccSupport() const
{
	return (firmwareVersion() >= XsVersion(1, 1, 0));
}

uint32_t Mti6X0Device::supportedStatusFlags() const
{
	return (uint32_t) (0
		| (deviceId().isImu() ? 0 : XSF_OrientationValid
			|XSF_NoRotationMask
			|XSF_RepresentativeMotion
			)
		|XSF_ExternalClockSynced
		|XSF_ClipAccX
		|XSF_ClipAccY
		|XSF_ClipAccZ
		|XSF_ClipGyrX
		|XSF_ClipGyrY
		|XSF_ClipGyrZ
		|XSF_ClipMagX
		|XSF_ClipMagY
		|XSF_ClipMagZ
		//|XSF_Retransmitted
		|XSF_ClippingDetected
		//|XSF_Interpolated
		|XSF_SyncIn
		|XSF_SyncOut
		//|XSF_FilterMode
		//|XSF_HaveGnssTimePulse
		);
}
