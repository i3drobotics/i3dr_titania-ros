#include <VimbaCPP/Include/VimbaCPP.h>
#include "iostream"

using namespace AVT::VmbAPI;

void listCameras(){
    // Initalise vimba systems api
    VimbaSystem& vimba_system(VimbaSystem::GetInstance());
    VmbErrorType err_startup = vimba_system.Startup();
    if( err_startup != VmbErrorSuccess ){
        std::cout << "Could not start vimba. Error code: " << err_startup << std::endl;
        return;
    }
    // List available cameras
    CameraPtrVector cameras;
    VmbErrorType err_get_cam = vimba_system.GetCameras( cameras );            // Fetch all cameras known to Vimba
    if( err_get_cam != VmbErrorSuccess ){
        std::cout << "Could not list cameras. Error code: " << err_get_cam << std::endl;
        return;
    }
    std::cout << "Cameras found: " << cameras.size() << std::endl;
    vimba_system.Shutdown();
}