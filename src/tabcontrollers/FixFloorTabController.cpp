#include "FixFloorTabController.h"
#include <QQuickWindow>
#include "../overlaycontroller.h"
#include <fstream>


using namespace std;

vector<string> split( const string& s, char delim )
{
    vector<string> elems;
    string item;
    for ( char ch : s )
    {
        if ( ch == delim )
        {
            if ( !item.empty() )
                elems.push_back( item );
            item.clear();
        }
        else
        {
            item += ch;
        }
    }
    if ( !item.empty() )
        elems.push_back( item );
    return elems;
}

void rotateCoordinates( double coordinates[3], double angle );

// application namespace
namespace advsettings
{
void FixFloorTabController::initStage2( OverlayController* var_parent )
{
    this->parent = var_parent;
}

void FixFloorTabController::dashboardLoopTick(
    vr::TrackedDevicePose_t* devicePoses )
{
    if ( state > 0 )
    {
        if ( measurementCount == 0 )
        {
            // Get Controller ids for left/right hand
            auto leftId
                = vr::VRSystem()->GetTrackedDeviceIndexForControllerRole(
                    vr::TrackedControllerRole_LeftHand );
            if ( leftId == vr::k_unTrackedDeviceIndexInvalid )
            {
                statusMessage = "No left controller found.";
                statusMessageTimeout = 2.0;
                emit statusMessageSignal();
                emit measureEndSignal();
                state = 0;
                return;
            }
            auto rightId
                = vr::VRSystem()->GetTrackedDeviceIndexForControllerRole(
                    vr::TrackedControllerRole_RightHand );
            if ( rightId == vr::k_unTrackedDeviceIndexInvalid )
            {
                statusMessage = "No right controller found.";
                statusMessageTimeout = 2.0;
                emit statusMessageSignal();
                emit measureEndSignal();
                state = 0;
                return;
            }
            // Get poses
            vr::TrackedDevicePose_t* leftPose = devicePoses + leftId;
            vr::TrackedDevicePose_t* rightPose = devicePoses + rightId;
            if ( !leftPose->bPoseIsValid || !leftPose->bDeviceIsConnected
                 || leftPose->eTrackingResult != vr::TrackingResult_Running_OK )
            {
                statusMessage = "Left controller tracking problems.";
                statusMessageTimeout = 2.0;
                emit statusMessageSignal();
                emit measureEndSignal();
                state = 0;
                return;
            }
            else if ( !rightPose->bPoseIsValid || !rightPose->bDeviceIsConnected
                      || rightPose->eTrackingResult
                             != vr::TrackingResult_Running_OK )
            {
                statusMessage = "Right controller tracking problems.";
                statusMessageTimeout = 2.0;
                emit statusMessageSignal();
                emit measureEndSignal();
                state = 0;
                return;
            }
            else
            {
                // The controller with the lowest y-pos is the floor fix
                // reference

                if ( leftPose->mDeviceToAbsoluteTracking.m[1][3]
                     < rightPose->mDeviceToAbsoluteTracking.m[1][3] )
                {
                    referenceController = leftId;
                }
                else
                {
                    referenceController = rightId;
                }

                auto& m = devicePoses[referenceController]
                              .mDeviceToAbsoluteTracking.m;
                tempOffsetX = static_cast<double>( m[0][3] );
                tempOffsetY = static_cast<double>( m[1][3] );
                tempOffsetZ = static_cast<double>( m[2][3] );

                /*
                | Intrinsic y-x'-z" rotation matrix:
                | cr*cy+sp*sr*sy | cr*sp*sy-cy*sr | cp*sy |
                | cp*sr          | cp*cr          |-sp    |
                | cy*sp*sr-cr*sy | cr*cy*sp+sr*sy | cp*cy |

                yaw = atan2(cp*sy, cp*cy) [pi, -pi], CCW
                pitch = -asin(-sp) [pi/2, -pi/2]
                roll = atan2(cp*sr, cp*cr) [pi, -pi], CW
                */
                tempRoll = std::atan2( static_cast<double>( m[1][0] ),
                                       static_cast<double>( m[1][1] ) );
                measurementCount = 1;
            }
        }
        else
        {
            measurementCount++;
            auto& m
                = devicePoses[referenceController].mDeviceToAbsoluteTracking.m;

            double rollDiff = std::atan2( static_cast<double>( m[1][0] ),
                                          static_cast<double>( m[1][1] ) )
                              - tempRoll;
            if ( rollDiff > M_PI )
            {
                rollDiff -= 2.0 * M_PI;
            }
            else if ( rollDiff < -M_PI )
            {
                rollDiff += 2.0 * M_PI;
            }
            tempRoll += rollDiff / static_cast<double>( measurementCount );
            if ( tempRoll > M_PI )
            {
                tempRoll -= 2.0 * M_PI;
            }
            else if ( tempRoll < -M_PI )
            {
                tempRoll += 2.0 * M_PI;
            }

            if ( measurementCount >= 25 )
            {
                int type = 0;
                if ( std::abs( tempRoll ) <= M_PI_2 )
                {
                    type = getControllerType( referenceController );
                    if ( type == Controller_Wand )
                    {
                        floorOffsetY = static_cast<float>( tempOffsetY )
                                       - controllerUpOffsetCorrection;
                    }
                    else if ( type == Controller_Knuckles )
                    {
                        // Production

                        floorOffsetY = static_cast<float>( tempOffsetY )
                                       - knucklesUpOffsetCorrection;

                        // Generate Offset
                        // Comment out production and un-comment this.
                        // Only if Floor is known to be good (zero with wand)

                        /*
                        floorOffsetY = static_cast<float>(tempOffsetY);
                        LOG(INFO) << "Offset For Knuckles up is: " <<
                        floorOffsetY;
                        */
                    }
                    else
                    {
                        floorOffsetY = static_cast<float>( tempOffsetY );
                    }
                }
                else
                {
                    type = getControllerType( referenceController );
                    if ( type == Controller_Wand )
                    {
                        floorOffsetY = static_cast<float>( tempOffsetY )
                                       - controllerDownOffsetCorrection;
                    }
                    else if ( type == Controller_Knuckles )
                    {
                        // Production

                        floorOffsetY = static_cast<float>( tempOffsetY )
                                       - knucklesDownOffsetCorrection;

                        // Generate Offset
                        // Comment out production and un-comment this.
                        // Only if Floor is known to be good (zero with wand)

                        /*
                        floorOffsetY = static_cast<float>(tempOffsetY);
                        LOG(INFO) << "Offset For Knuckles up is: " <<
                        floorOffsetY;
                        */
                    }
                    else
                    {
                        floorOffsetY = static_cast<float>( tempOffsetY );
                    }
                }

                floorOffsetX = static_cast<float>( tempOffsetX );
                floorOffsetZ = static_cast<float>( tempOffsetZ );

                LOG( INFO )
                    << "Fix Floor and adjust space center: Floor Offset = ["
                    << floorOffsetX << ", " << floorOffsetY << ", "
                    << floorOffsetZ << "]";


                // 原点位置オフセットを読み込む
                double offsetF[3];
                std::ifstream ifs( "c:\\offset.txt" );
                std::string str;

                getline( ifs, str );
                const auto floats = split( str, ',' );
                offsetF[0] = atof( floats[0].c_str() );
                offsetF[1] = atof( floats[1].c_str() );
                offsetF[2] = atof( floats[2].c_str() );

                getline( ifs, str );
                const auto ints = split( str, ',' );
                auto i0 = atoi( ints[0].c_str() );
                auto i1 = atoi( ints[1].c_str() );
                auto i2 = atoi( ints[2].c_str() );
                auto i3 = atoi( ints[3].c_str() );
                // Quest2 Touchコントローラーを床に置いたときの向きを取得
                double controllerRotDeg
                    = std::atan2( m[i0][i1], m[i2][i3] ) / M_PI * 180.0;

                double fr = controllerRotDeg + atof( floats[3].c_str() );


                // 後で空間を回転する反対にオフセット位置を回転させる
                rotateCoordinates( offsetF, -fr/180.0*M_PI );

                float offset[3] = { 0, 0, 0 };
                offset[1] = floorOffsetY + offsetF[1];
                if ( state == 2 )
                {
                    offset[0] = floorOffsetX + offsetF[0];
                    offset[2] = floorOffsetZ + offsetF[2];
                }

                parent->AddOffsetToUniverseCenter(
                    vr::TrackingUniverseStanding, offset, true );
                statusMessage = ( state == 2 ) ? "Recentering ... Ok"
                                               : "Fixing ... OK";
                statusMessageTimeout = 1.0;
                emit statusMessageSignal();
                emit measureEndSignal();
                setCanUndo( true );
                state = 0;
                parent->m_moveCenterTabController.zeroOffsets();
                // this reset fixes a bug where fixed floor wouldn't show in
                // WMR
                parent->m_moveCenterTabController.reset();

                // 空間を回転させる HMDを中心にまわすモードだとうまくいかない
                parent->m_moveCenterTabController.setRotation( fr  * 100 );
            }
        }
    }
}

int FixFloorTabController::getControllerType(
    vr::TrackedDeviceIndex_t controllerRole )
{
    const uint32_t maxLength = 64;
    /*
    vr::k_unMaxPropertyStringSize is Theoretical Max Size, however it is
    1024*32, and its pretty unrealistic to expect a controller name to be that
    big. We are just going to set 64 as an arbitrary size, and print error to
    log if too small
    */
    char controllerTypeString[maxLength];
    vr::ETrackedPropertyError error;
    auto stringLength = vr::VRSystem()->GetStringTrackedDeviceProperty(
        controllerRole,
        vr::Prop_ControllerType_String,
        controllerTypeString,
        maxLength,
        &error );
    if ( error != vr::TrackedProp_Success )
    {
        LOG( ERROR ) << "Error With Controller Type: "
                     << vr::VRSystem()->GetPropErrorNameFromEnum( error );
    }
    else if ( stringLength == 0 )
    {
        LOG( ERROR ) << "Device Index not valid";
    }
    else if ( strcmp( controllerTypeString, "knuckles" ) == 0 )
    {
        return Controller_Knuckles;
    }
    else if ( strcmp( controllerTypeString, "vive_controller" ) == 0 )
    {
        return Controller_Wand;
    }
    return Controller_Unknown;
}

QString FixFloorTabController::currentStatusMessage()
{
    return statusMessage;
}

float FixFloorTabController::currentStatusMessageTimeout()
{
    return statusMessageTimeout;
}

bool FixFloorTabController::canUndo() const
{
    return m_canUndo;
}

void FixFloorTabController::setCanUndo( bool value, bool notify )
{
    if ( m_canUndo != value )
    {
        m_canUndo = value;
        if ( notify )
        {
            emit canUndoChanged( m_canUndo );
        }
    }
}

void FixFloorTabController::fixFloorClicked()
{
    parent->m_moveCenterTabController.reset();
    statusMessage = "Fixing ...";
    statusMessageTimeout = 1.0;
    emit statusMessageSignal();
    emit measureStartSignal();
    measurementCount = 0;
    state = 1;
}

void FixFloorTabController::recenterClicked()
{
    parent->m_moveCenterTabController.reset();
    statusMessage = "Fixing ...";
    statusMessageTimeout = 1.0;
    emit statusMessageSignal();
    emit measureStartSignal();
    measurementCount = 0;
    state = 2;
}

void FixFloorTabController::undoFixFloorClicked()
{
    parent->m_moveCenterTabController.reset();
    parent->AddOffsetToUniverseCenter(
        vr::TrackingUniverseStanding, 0, -floorOffsetX, false );
    parent->AddOffsetToUniverseCenter(
        vr::TrackingUniverseStanding, 1, -floorOffsetY, false );
    parent->AddOffsetToUniverseCenter(
        vr::TrackingUniverseStanding, 2, -floorOffsetZ, false );
    LOG( INFO ) << "Fix Floor: Undo Floor Offset = [" << -floorOffsetX << ", "
                << -floorOffsetY << ", " << -floorOffsetZ << "]";
    floorOffsetY = 0.0f;
    statusMessage = "Undo ... OK";
    statusMessageTimeout = 1.0;
    emit statusMessageSignal();
    setCanUndo( false );
    parent->m_moveCenterTabController.zeroOffsets();
}

} // namespace advsettings
