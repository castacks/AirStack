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

//== INCLUDES =================================================================----

#include <winsock.h>

//== GLOBAL DEFINITIONS AND SETTINGS ==========================================----

namespace
{
    const int kMaxAddressLength = 128;
    const int kSubPacketMaxSize = 1400;
}

//== CLASS DEFINITION =========================================================----

class NATNET_API cSlipStream
{
public:
    cSlipStream( const char *Address, int Port );
    ~cSlipStream();

    ///<summary>Output a block of data over UDP.</summary>
    bool Stream( unsigned char* Buffer, int BufferSize );

private:
    char   mAddress[kMaxAddressLength];
    int    mPort;

    SOCKET mSocket;

    bool   StreamPacket( unsigned char *Buffer, int BufferSize );
};
