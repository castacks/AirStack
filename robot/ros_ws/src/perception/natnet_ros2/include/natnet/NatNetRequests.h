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


// Parameters: (none)
// Response: float32_t value representing current system's unit scale in terms of millimeters.
#define NATNET_REQUEST_GETUNITSTOMILLIMETERS "UnitsToMillimeters"

// Parameters: (none)
// Response: float32_t value representing current system's tracking frame rate in frames per second.
#define NATNET_REQUEST_GETFRAMERATE "FrameRate"

// Parameters: (none)
// Response: int32_t value representing the current mode. (0 = Live, 1 = Recording, 2 = Edit)
#define NATNET_REQUEST_GETCURRENTMODE "CurrentMode"

// Parameters: (none)
// Response: (none)
#define NATNET_REQUEST_STARTRECORDING "StartRecording"

// Parameters: (none)
// Response: (none)
#define NATNET_REQUEST_STOPRECORDING "StopRecording"

// Parameters: (none)
// Response: (none)
#define NATNET_REQUEST_SWITCHTOLIVEMODE "LiveMode"

// Parameters: (none)
// Response: (none)
#define NATNET_REQUEST_SWITCHTOEDITMODE "EditMode"

// Parameters: (none)
// Response: (none)
#define NATNET_REQUEST_TIMELINEPLAY "TimelinePlay"

// Parameters: (none)
// Response: (none)
#define NATNET_REQUEST_TIMELINESTOP "TimelineStop"

// Parameters: New playback take name.
// Response: (none)
#define NATNET_REQUEST_SETPLAYBACKTAKENAME "SetPlaybackTakeName"

// Parameters: New record take name.
// Response: (none)
#define NATNET_REQUEST_SETRECORDTAKENAME "SetRecordTakeName"

// Parameters: Session path/name to either switch to or to create.
// Response: int32_t value indicating whether the command succeeded. Zero if successful.
#define NATNET_REQUEST_SETCURRENTSESSION "SetCurrentSession"

// Parameters: (none)
// Response: Unix-style path of the current session as a string, including trailing delimiter.
#define NATNET_REQUEST_CURRENTSESSIONPATH "CurrentSessionPath"

// Parameters: The new start frame for the playback range.
// Response: (none)
#define NATNET_REQUEST_SETPLAYBACKSTARTFRAME "SetPlaybackStartFrame"

// Parameters: The new end frame for the playback range.
// Response: (none)
#define NATNET_REQUEST_SETPLAYBACKSTOPFRAME "SetPlaybackStopFrame"

// Parameters: The new current frame for playback.
// Response: (none)
#define NATNET_REQUEST_SETPLAYBACKCURRENTFRAME "SetPlaybackCurrentFrame"

// Parameters: One to turn playback looping on. Zero to turn looping off.
// Response: (none)
#define NATNET_REQUEST_SETPLAYBACKLOOPING "SetPlaybackLooping"

// Parameters: Node (asset) name (case sensitive)
// Response: int32_t value indicating whether the command successfully enabled the asset. Zero if successful.
// Example command string: "EnableAsset,Rigid Body 1"
#define NATNET_REQUEST_ENABLEASSET "EnableAsset"

// Parameters: Node (asset) name (case sensitive)
// Response: int32_t value indicating whether the command successfully disabled the asset. Zero if successful.
#define NATNET_REQUEST_DISABLEASSET "DisableAsset"

// Parameters: Node name (or empty to query application properties) and property name. 
// Response: String representation of the property value.
// Example command string: "GetProperty,,MoodLiveColor"
#define NATNET_REQUEST_GETPROPERTY "GetProperty"

// Parameters: Node name (if applicable leave it empty if not), property name, and desired value.
// Response: int32_t value indicating whether the command successfully updated the data. Zero if successful.
// Example command string: "SetProperty,,Unlabeled Markers,False"
#define NATNET_REQUEST_SETPROPETRY "SetProperty"

// Parameters: Take name, or empty string to query the current take.
// Response: String representation of the property value.
#define NATNET_REQUEST_GETTAKEPROPERTY "GetTakeProperty"

// Parameters: (none)
// Response: int32_t value representing length of current take in frames.
#define NATNET_REQUEST_GETCURRENTTAKELENGTH "CurrentTakeLength"

// Parameters: Node (asset) name (case sensitive)
// Response: int32_t value indicating whether the command successfully recalibrated the asset. Zero if successful.
// Example command string: "RecalibrateAsset,Skeleton1"
#define NATNET_REQUEST_RECALIBRATEASSET "RecalibrateAsset"

// Parameters: Node (asset) name (case sensitive)
// Response: int32_t value indicating whether the command successfully reoriented the asset. Zero if successful.
// Example command string: "ResetAssetOrientation,RigidBody1"
#define NATNET_REQUEST_RESETASSETORIENTATION "ResetAssetOrientation"

// Parameters: Node (asset) name (case sensitive)
// Response: int32_t value indicating whether the command successfully reoriented the asset. Zero if successful.
// Example command string: "ResetAssetLocation,RigidBody1"
#define NATNET_REQUEST_RESETASSETLOCATION "ResetAssetLocation"