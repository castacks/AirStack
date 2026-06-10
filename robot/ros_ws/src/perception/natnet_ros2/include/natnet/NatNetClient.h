//=============================================================================
// Copyright © 2025 NaturalPoint, Inc. All Rights Reserved.
// 
// THIS SOFTWARE IS GOVERNED BY THE OPTITRACK PLUGINS EULA AVAILABLE AT https://www.optitrack.com/about/legal/eula.html 
// AND/OR FOR DOWNLOAD WITH THE APPLICABLE SOFTWARE FILE(S) (“PLUGINS EULA”). BY DOWNLOADING, INSTALLING, ACTIVATING 
// AND/OR OTHERWISE USING THE SOFTWARE, YOU ARE AGREEING THAT YOU HAVE READ, AND THAT YOU AGREE TO COMPLY WITH AND ARE
//  BOUND BY, THE PLUGINS EULA AND ALL APPLICABLE LAWS AND REGULATIONS. IF YOU DO NOT AGREE TO BE BOUND BY THE PLUGINS
//  EULA, THEN YOU MAY NOT DOWNLOAD, INSTALL, ACTIVATE OR OTHERWISE USE THE SOFTWARE AND YOU MUST PROMPTLY DELETE OR
//  RETURN IT. IF YOU ARE DOWNLOADING, INSTALLING, ACTIVATING AND/OR OTHERWISE USING THE SOFTWARE ON BEHALF OF AN ENTITY,
//  THEN BY DOING SO YOU REPRESENT AND WARRANT THAT YOU HAVE THE APPROPRIATE AUTHORITY TO ACCEPT THE PLUGINS EULA ON
//  BEHALF OF SUCH ENTITY. See license file in root directory for additional governing terms and information.
//=============================================================================

#pragma once

#include "NatNetTypes.h"


// NatNetClient is a complete C++ class for connecting to NatNet server
// applications such as OptiTrack Motive.
class NATNET_API NatNetClient
{
public:
    NatNetClient();
    ~NatNetClient();

    ErrorCode Connect( const sNatNetClientConnectParams& connectParams );
    ErrorCode Disconnect();

    ErrorCode SetFrameReceivedCallback( NatNetFrameReceivedCallback pfnDataCallback, void* pUserContext = NULL );
    ErrorCode SetUnknownMessageCallback( NatNetUnknownMessageCallback pfnMsgCallback, void* pUserContext = NULL );

    ErrorCode SendMessageAndWait( const char* szRequest, void** ppServerResponse, int* pResponseSize );
    ErrorCode SendMessageAndWait( const char* szRequest, int tries, int timeout, void** ppServerResponse, int* pResponseSize );

    ErrorCode GetServerDescription( sServerDescription* pServerDescription );
    ErrorCode GetDataDescriptionList( sDataDescriptions** ppDataDescriptions, uint32_t descriptionTypesMask = 0xFFFFFFFF );

    ErrorCode GetPredictedRigidBodyPose(int32_t streamingId, sRigidBodyData& outRbPose, double dt = 0);

    double SecondsSinceHostTimestamp( uint64_t hostTimestamp ) const;

    //////////////////////////////////////////////////////////////////////////
    // Deprecated methods

    NATNET_DEPRECATED( "The constructor specifying connection type is deprecated; use the sNatNetClientConnectParams::connectionType field instead." )
    explicit NatNetClient( int connectionType );

    NATNET_DEPRECATED( "This method is deprecated; use Connect instead." )
    ErrorCode Initialize( const char* szLocalAddress, const char* szServerAddress, int HostCommandPort = 0, int HostDataPort = 0 );

    NATNET_DEPRECATED( "This method has been renamed; use Disconnect instead." )
    ErrorCode Uninitialize()
    {
        return Disconnect();
    }

    NATNET_DEPRECATED( "This method has been renamed; use SetFrameReceivedCallback instead." )
    ErrorCode SetDataCallback( void (*CallbackFunction)(sFrameOfMocapData* pFrameOfData, void* pUserData), void* pUserData = NULL )
    {
        return SetFrameReceivedCallback( CallbackFunction, pUserData );
    }

    NATNET_DEPRECATED( "This method is deprecated; the log callback is installed globally, and should be set using the function NatNet_SetLogCallback from NatNetCAPI.h instead." )
    int SetMessageCallback( void (*CallbackFunction)(int id, char* szTraceMessage) );

    NATNET_DEPRECATED( "This method is deprecated; use GetDataDescriptionList, which returns an ErrorCode instead of the number of descriptions (which is still available as sDataDescriptions::nDataDescriptions)." )
    int GetDataDescriptions( sDataDescriptions** ppDataDescriptions );

    NATNET_DEPRECATED( "This functionality is no longer supported by Motive servers." )
    sFrameOfMocapData* GetLastFrameOfData();

    NATNET_DEPRECATED( "This method is deprecated; use the function NatNet_GetVersion from NatNetCAPI.h instead." )
    void NatNetVersion( unsigned char Version[4] );

    NATNET_DEPRECATED( "This method is obsolete and does nothing; implement filtering logic in your log callback handler instead." )
    void SetVerbosityLevel( int iLevel );

    NATNET_DEPRECATED( "This method is deprecated; use the function NatNet_DecodeTimecode from NatNetCAPI.h instead." )
    bool DecodeTimecode( unsigned int timecode, unsigned int timecodeSubframe, int* pOutHour, int* pOutMinute, int* pOutSecond, int* pOutFrame, int* pOutSubframe );

    NATNET_DEPRECATED( "This method is deprecated; use the function NatNet_TimecodeStringify from NatNetCAPI.h instead." )
    bool TimecodeStringify( unsigned int timecode, unsigned int timecodeSubframe, char* outBuffer, int outBufferSize );

    NATNET_DEPRECATED( "This method is deprecated; use the function NatNet_DecodeID from NatNetCAPI.h instead." )
    void DecodeID( unsigned int compositeId, int* pOutEntityId, int* pOutMemeberId );

    NATNET_DEPRECATED( "This method is deprecated; use the function NatNet_CopyFrame from NatNetCAPI.h instead." )
    void CopyFrame( sFrameOfMocapData* pSrc, sFrameOfMocapData* pDst );

    NATNET_DEPRECATED( "This method is deprecated; use the function NatNet_FreeFrame from NatNetCAPI.h instead." )
    void FreeFrame( sFrameOfMocapData* pFrame );

    NATNET_DEPRECATED("This method is deprecated; use the function NatNet_FreeDescriptions from NatNetCAPI.h instead.")
    void FreeDescriptions( sDataDescriptions* pDesc );

private:
    class ClientCore* m_pClientCore;
};
